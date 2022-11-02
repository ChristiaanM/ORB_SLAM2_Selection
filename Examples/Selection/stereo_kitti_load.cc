/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

#include "Osmap.h"

#include "ExampleUtils.h"

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);





int main(int argc, char **argv)
{
    std::cout << "args " << argc << std::endl;
    
    if (argc != 7 && argc != 9)
    {
        cerr << endl
             << "Usage: ./stereo_kitti_load path_to_vocabulary path_to_settings path_to_sequence path_to_mapyaml start_id mode (end_id) (output_map)" << endl;
        return 1;
    }

    int startId = stoi(argv[5]);


    SlamMode slam_mode;
    size_t BAiters; 
    parseMode(argv[6],slam_mode,BAiters);    

   
    int saveId = 0;
    string outputMap = string();
    if (argc == 9)
    {
        saveId = stoi(argv[7]);
        outputMap = string(argv[8]);
    }
    
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    
    SelectionSettings ssettings;
    if (slam_mode == SlamMode::cull)
        ssettings.kfCullLoaded = true;

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true,ssettings, BAiters);
    ORB_SLAM2::Osmap osmap(SLAM,1226,370);
    osmap.mapLoadTracking(argv[4]);

    if (slam_mode == SlamMode::vo || slam_mode ==  SlamMode::local)
        SLAM.ActivateLocalizationMode();
    if (slam_mode == SlamMode::sequential)
        SLAM.PartialShutdown();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    cv::Mat imLeft, imRight;
    for (int ni = startId; ni < nImages && (ni < saveId || !(saveId)); ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imLeft.empty())
        {
            cerr << endl
                 << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        if (slam_mode == SlamMode::vo)
        {
            SLAM.mpTracker->mState = ORB_SLAM2::Tracking::LOST;
            SLAM.mpTracker->mnLastRelocFrameId = 0;
        }
        if (slam_mode == SlamMode::sequential)
        {
            SLAM.SequentialRunSetup();
            SLAM.TrackStereo(imLeft, imRight, tframe);
            SLAM.SequentialRun();
        }
        else 
            SLAM.TrackStereo(imLeft, imRight, tframe);

        

        

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);

        if (slam_mode == SlamMode::bench)
            SLAM.WaitForLoopGBA();
    }


    if (argc == 9)
    {
        ORB_SLAM2::Osmap osmap(SLAM,1226,370);
        osmap.mapSaveAndShutdown(outputMap);
    }
    
    else
    {
        // Stop all threads
        SLAM.Shutdown();
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
        << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;
    
    //SlamMode slam_mode = (SlamMode) mode;

    string output_filename = generateTrajectoryFilename(argv[4],slam_mode);
    SLAM.SaveTrajectoryTUM(output_filename + ".traject");
    SLAM.SaveTrajectoryKITTI(output_filename  + ".kitti_traject");
    SLAM.SaveLoopCandidates(output_filename + ".loop");

    /*for(size_t i=0;i<1;i++)
    {
        SLAM.GlobalBA(20);
        string output_filename2 = generateTrajectoryFilename(argv[4],slam_mode);
        SLAM.SaveTrajectoryTUM(output_filename2 + ".traject");
        SLAM.SaveTrajectoryKITTI(output_filename2  + ".kitti_traject");
        SLAM.SaveLoopCandidates(output_filename2 + ".loop");
    }*/
       
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

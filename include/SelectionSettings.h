#ifndef SELECTIONSETTINGS_H
#define SELECTIONSETTINGS_H

#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>





struct SelectionSettings
{

    // KEYFRAME INSERTION SETTING

    bool kfAlwaysInsert = false;
    
    double kfMatchesRatio = 0.75f; // Minimum number of tracked features vs non-tracked features
    int kfCloseMaxTracked = 100; // Insert a close keyframe if less than this number of close points are tracked
    int kfCloseMinNonTracked = 70; // and more than this number is available. 

    double kfCullPointThresh = 3;
    double kfCullRatio = 0.9f;
    
    bool kfCullLoaded = false;
    
    double mpCullVisRatio = 0.25f;

    void writeToFile(const std::string &filename)
    {
        cv::FileStorage fs(filename,cv::FileStorage::WRITE);
        #define SSETTINGS_WRITE(VAR) fs << #VAR << VAR
        SSETTINGS_WRITE(kfAlwaysInsert);
        SSETTINGS_WRITE(kfMatchesRatio);
        SSETTINGS_WRITE(kfCloseMaxTracked);
        SSETTINGS_WRITE(kfCloseMinNonTracked);
        SSETTINGS_WRITE(kfCullPointThresh);
        SSETTINGS_WRITE(kfCullRatio);
        SSETTINGS_WRITE(kfCullLoaded);
        SSETTINGS_WRITE(mpCullVisRatio);
        fs.release();
    }


    static SelectionSettings readFromFile(const std::string &filename)
    {
        SelectionSettings out;
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        #define SSETTINGS_READ_NONEMPTY(VAR) if (!fs[#VAR].empty()) fs[#VAR] >> out.VAR
        SSETTINGS_READ_NONEMPTY(kfAlwaysInsert);
        SSETTINGS_READ_NONEMPTY(kfMatchesRatio);
        SSETTINGS_READ_NONEMPTY(kfCloseMaxTracked);
        SSETTINGS_READ_NONEMPTY(kfCloseMinNonTracked);
        SSETTINGS_READ_NONEMPTY(kfCullPointThresh);
        SSETTINGS_READ_NONEMPTY(kfCullRatio);
        SSETTINGS_READ_NONEMPTY(kfCullLoaded);
        SSETTINGS_READ_NONEMPTY(mpCullVisRatio);
        fs.release();

        return out;
    }

    void print()
    {
        #define SSETTINGS_OUPUT(VAR) std::cout << #VAR << " : " <<  VAR << std::endl
        SSETTINGS_OUPUT(kfAlwaysInsert);
        SSETTINGS_OUPUT(kfMatchesRatio);
        SSETTINGS_OUPUT(kfCloseMaxTracked);
        SSETTINGS_OUPUT(kfCloseMinNonTracked);
        SSETTINGS_OUPUT(kfCullPointThresh);
        SSETTINGS_OUPUT(kfCullRatio);
        SSETTINGS_OUPUT(kfCullLoaded);
        SSETTINGS_OUPUT(mpCullVisRatio);
    }


};

  



#endif
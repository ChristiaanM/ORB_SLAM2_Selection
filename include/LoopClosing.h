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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

//#define LOWER_LOOP_THRESH

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;

//#define YAML_LOOP_LOGS
struct LoopMatch
{
    size_t mPointId;
    size_t mFound;
    float mRatio;
    size_t mObs; 

    bool mBowMatch;
    bool mRansacMatch;
    bool mSearchMatch;
    size_t mWord;
    size_t mWordCnt;
    size_t mClosest;
    

};
struct LogEntryLoopCandidate
{
    size_t mnFrameId = 0;
    size_t mnKFId = 0;
    size_t mnMatchId = 0;
    size_t mnMatchKFId = 0;

    size_t mnFeaturesBow = 0;
    size_t mnFeaturesRansac = 0;
    size_t mnFeaturesOpt = 0;
    size_t mnFeaturesLocal = 0;

    KeyFrame *match_keyframe = nullptr;
    //std::vector<MapPoint*> bowmappoints;
    //std::vector<MapPoint*> ransacmappoints;
    std::vector<LoopMatch> loopMatches;
};


 



class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale, size_t BAiters);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF, size_t iterations);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    void saveLog(const string &filename);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    void CorrectLoop();

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    bool mnFullBAIdx;

    // Loop-closure Data Log
    std::deque<LogEntryLoopCandidate> mvLoopCandidateLog;


    size_t mBAiters;
    #ifdef LOWER_LOOP_THRESH 
    size_t mnLoopBowTh = 10;
    size_t mnLoopRansacTh = 10;
    size_t mnLoopOptTh = 10;
    size_t mnLoopLocalTh = 20;
    #else
    size_t mnLoopBowTh = 20;
    size_t mnLoopRansacTh = 20;
    size_t mnLoopOptTh = 20;
    size_t mnLoopLocalTh = 40;
    #endif

    
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H

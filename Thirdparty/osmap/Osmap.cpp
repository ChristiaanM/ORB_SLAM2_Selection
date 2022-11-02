/**
* This file is part of OSMAP.
*
* Copyright (C) 2018-2019 Alejandro Silvestri <alejandrosilvestri at gmail>
* For more information see <https://github.com/AlejandroSilvestri/osmap>
*
* OSMAP is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OSMAP is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OSMAP. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fstream>
#include <iostream>
#include <assert.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/map.h>

#include "Osmap.h"

// Option check macro
#define OPTION(OP)   \
	if (options[OP]) \
		headerFile << #OP;

// Log variable
#define LOGV(VAR) \
	if (verbose)  \
		cout << #VAR << ": " << VAR << endl;

using namespace std;
using namespace cv;

namespace ORB_SLAM2
{

Osmap::Osmap(System &_system, size_t cam_x,size_t cam_y) : map(static_cast<OsmapMap &>(*_system.mpMap)),
								keyFrameDatabase(*_system.mpKeyFrameDatabase),
								system(_system),
								currentFrame(_system.mpTracker->mCurrentFrame),
								camPx(cam_x),
								camPy(cam_y)

{
#ifndef OSMAP_DUMMY_MAP

	/* Every new MapPoint require a dummy pRefKF in its constructor, copying the following parameters:
	 *
	 * - mnFirstKFid(pRefKF->mnId)
	 * - mnFirstFrame(pRefKF->mnFrameId)
	 * - mpRefKF(pRefKF)
	 *
	 * A fake keyframe construction requires a Frame with
	 *
	 *  - mTcw (must be provided, error if not)
	 *  - Grid (already exists)
	 */
	Frame dummyFrame;
	dummyFrame.mTcw = Mat::eye(4, 4, CV_32F);
	dummyFrame.mnMaxX = camPx;
	dummyFrame.mnMaxY = camPy;
	dummyFrame.mnMinX = 0;
	dummyFrame.mnMinY = 0;
	dummyFrame.mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(dummyFrame.mnMaxX-dummyFrame.mnMinX);
    dummyFrame.mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(dummyFrame.mnMaxY-dummyFrame.mnMinY);

	pRefKF = new KeyFrame(dummyFrame, &map, &keyFrameDatabase);

#endif
};

void Osmap::mapSave(const string givenFilename, bool pauseThreads)
{
	

	// Stop threads
	if (pauseThreads)
	{
		system.mpLocalMapper->RequestStop();
		while (!system.mpLocalMapper->isStopped())
			usleep(1000);
	}
	

	// Strip out .yaml if present
	string baseFilename, filename, pathDirectory;
	parsePath(givenFilename, &filename, &pathDirectory);
	if (pathDirectory != "")
		chdir(pathDirectory.c_str());

	int length = filename.length();
	if (length > 5 && filename.substr(length - 5) == ".yaml")
		baseFilename = filename.substr(length - 5);
	else
		baseFilename = filename;

	// Map depuration
	if (!options[NO_DEPURATION])
		depurate();

	// Actual saving
	filename = baseFilename + ".yaml";

	// Open YAML file for write, it will be the last file to close.
	// FileStorage https://docs.opencv.org/3.1.0/da/d56/classcv_1_1FileStorage.html
	FileStorage headerFile(filename, FileStorage::WRITE);
	if (!headerFile.isOpened())
	{
		// Is this necessary?
		cerr << "Couldn't create file " << baseFilename << ".yaml, map not saved." << endl;
		return;
	}

	// MapPoints
	if (!options[NO_MAPPOINTS_FILE])
	{
		// Order mappoints by mnId
		getMapPointsFromMap();

		// New file
		filename = baseFilename + ".mappoints";

		// Serialize
		cout << "Saving " << filename << endl;
		headerFile << "mappointsFile" << filename;
		headerFile << "nMappoints" << MapPointsSave(filename);
	}

	// K: grab camera calibration matrices.  Will be saved to yaml file later.
	if (!options[K_IN_KEYFRAME])
	{
		getVectorKFromKeyframes();

	}

	// KeyFrames
	if (!options[NO_KEYFRAMES_FILE])
	{
		getKeyFramesFromMap();

		// New file
		filename = baseFilename + ".keyframes";

		// Serialize
		cout << "Saving " << filename << endl;
		headerFile << "keyframesFile" << filename;
		headerFile << "nKeyframes" << KeyFramesSave(filename);
	}

	// Features
	if (!options[NO_FEATURES_FILE])
	{
		filename = baseFilename + ".features";
		cout << "Saving " << filename << endl;
		headerFile << "featuresFile" << filename;
		headerFile << "nFeatures" << featuresSave(filename);
	}


	{
		getTrackingDataFromTracker();

		filename = baseFilename + ".tracking";
		cout << "Saving " << filename << endl;
		headerFile << "trackingFile" << filename;
		headerFile << "nFrames" << trackingFramesSave(filename);
	}

	// Save options, as an int
	headerFile << "Options" << (int)options.to_ulong();
	// Options
	if (options.any())
	{
		headerFile << "Options descriptions"
				   << "[:";
		OPTION(NO_LOOPS)
		OPTION(NO_FEATURES_DESCRIPTORS)
		OPTION(K_IN_KEYFRAME)
		OPTION(ONLY_MAPPOINTS_FEATURES)
		OPTION(FEATURES_FILE_DELIMITED)
		OPTION(FEATURES_FILE_NOT_DELIMITED)
		OPTION(NO_MAPPOINTS_FILE)
		OPTION(NO_KEYFRAMES_FILE)
		OPTION(NO_FEATURES_FILE)
		headerFile << "]";
	}

	// K: camera calibration matrices, save to yaml at the end of file.
	if (!options[K_IN_KEYFRAME])
	{
		// Save K matrices in header file yaml
		headerFile << "cameraMatrices"
				   << "[";
		for(size_t i = 0; i < vectorK.size();i++)
		{
			auto pK = vectorK[i];
			auto baseline = vectorBaselines[i];
			headerFile << "{"
				<< "fx" << pK->at<float>(0, 0) << "fy" << pK->at<float>(1, 1) << "cx" << pK->at<float>(0, 2) << "cy" << pK->at<float>(1, 2);
			if (baseline)
				 headerFile << "baseline" << baseline;
			headerFile << "}";
		}
		
		
		headerFile << "]";
	}

	// Save yaml file
	headerFile.release();

	// Clear temporary vectors
	clearVectors();

	if (pauseThreads)
		system.mpViewer->Release();
}

void  Osmap::mapSaveAndShutdown(string basefilename)
{
   	system.mpLocalMapper->RequestStop();
    system.mpViewer->RequestStop();
    while(!system.mpViewer->isStopped() || !system.mpLocalMapper->isStopped() || system.mpLoopCloser->isRunningGBA())
        usleep(5000);

    mapSave(basefilename, false);
    cout << "Save Done" << endl;

    system.mpLocalMapper->Release();
    system.mpViewer->Release();

	system.Shutdown();



}

void Osmap::mapLoad(string yamlFilename, bool noSetBad, bool pauseThreads, bool loadTracking)
{
#ifndef OSMAP_DUMMY_MAP
	LOGV(system.mpTracker->mState)
	// Initialize currentFrame via calling GrabImageMonocular just in case, with a dummy image.
	if (system.mpTracker->mState == ORB_SLAM2::Tracking::NO_IMAGES_YET)
	{
		LOGV(system.mpTracker->mState);
		
		Mat m = Mat::zeros(camPy, camPx, CV_8U);
		if (system.mSensor == ORB_SLAM2::System::MONOCULAR)
			system.mpTracker->GrabImageMonocular(m, 0.0);
		else
			system.mpTracker->GrabImageStereo(m, m, 0.0);
	}
#endif

	if (pauseThreads)
	{
		// Reset the tracker to clean the map
		system.mpLocalMapper->Release(); // Release local mapper just in case it's stopped, because if it is stopped it can't be reset
		system.mpTracker->Reset();
		// Here the system is reset, state is NO_IMAGE_YET

		// Stop LocalMapping and Viewer
		system.mpLocalMapper->RequestStop();
		system.mpViewer->RequestStop();
		while (!system.mpLocalMapper->isStopped())
			usleep(1000);
		while (!system.mpViewer->isStopped())
			usleep(1000);
	}



	LOGV(system.mpLocalMapper->isStopped())
	LOGV(system.mpViewer->isStopped())

	string filename;
	int intOptions;

	// Open YAML
	std::cout << "reading" << yamlFilename << std::endl;

	cv::FileStorage headerFile(yamlFilename, cv::FileStorage::READ);

	// Options
	headerFile["Options"] >> intOptions;
	options = intOptions;

	// K
	if (!options[K_IN_KEYFRAME])
	{
		vectorK.clear();
		FileNode cameraMatrices = headerFile["cameraMatrices"];
		FileNodeIterator it = cameraMatrices.begin(), it_end = cameraMatrices.end();
		for (; it != it_end; ++it)
		{
			Mat *k = new Mat();
			*k = Mat::eye(3, 3, CV_32F);
			k->at<float>(0, 0) = (*it)["fx"];
			k->at<float>(1, 1) = (*it)["fy"];
			k->at<float>(0, 2) = (*it)["cx"];
			k->at<float>(1, 2) = (*it)["cy"];
			vectorK.push_back(k);
			vectorBaselines.push_back((double) (*it)["baseline"]);
		}
	}

	// Change directory
	string pathDirectory;
	parsePath(yamlFilename, NULL, &pathDirectory);
	if (pathDirectory != "")
		chdir(pathDirectory.c_str());

	// MapPoints
	vectorMapPoints.clear();
	if (!options[NO_MAPPOINTS_FILE])
	{
		headerFile["mappointsFile"] >> filename;
		MapPointsLoad(filename);
	}

	// KeyFrames
	vectorKeyFrames.clear();
	if (!options[NO_KEYFRAMES_FILE])
	{
		headerFile["keyframesFile"] >> filename;
		KeyFramesLoad(filename);
	}

	// Features
	if (!options[NO_FEATURES_FILE])
	{
		headerFile["featuresFile"] >> filename;
		cout << "Loading features from " << filename << " ..." << endl;
		featuresLoad(filename);
	}

	if(loadTracking)
	{
		headerFile["trackingFile"] >> filename;
		cout << "Loading tracking data from " << filename << " ..." << endl;
		trackingFramesLoad(filename);
	}
	
	// Close yaml file
	headerFile.release();

	// Rebuild
	rebuild(noSetBad);

	// Copy to map
	setMapPointsToMap();
	setKeyFramesToMap();
	if (loadTracking)
		setTrackingDataToTracker();
	else 
	{
#if !defined OSMAP_DUMMY_MAP && !defined OS1
		if (system.mpTracker->mlRelativeFramePoses.empty())
		{
			// Add dummy point to trajectory recorder to avoid errors.  The point is in the origin of the map's reference system.
			system.mpTracker->mlRelativeFramePoses.push_back(cv::Mat::eye(4, 4, CV_32F));
			system.mpTracker->mlpReferences.push_back(getKeyFrame(0));
			system.mpTracker->mlFrameTimes.push_back(0.0);
			system.mpTracker->mlbLost.push_back(true);
		}
#endif
	}

	// Release temporary vectors
	clearVectors();

#ifndef OSMAP_DUMMY_MAP
	// Lost state, the system must relocalize itself in the just loaded map.
	// system.mpTracker->mState = ORB_SLAM2::Tracking::LOST;
#endif

	if (pauseThreads)
	{
		// Resume threads

		// Reactivate viewer.  Do not reactivate localMapper because the system resumes in "only tracking" mode immediatly after loading.
		system.mpViewer->Release();

		// Tracking do this when going to LOST state.
		// Invoked after viewer.Release() because of mutex.
		system.mpFrameDrawer->Update(system.mpTracker);
	}
}


void Osmap::mapLoadTracking(string yamlFilename, bool loadTracking)
{
	cout << "Loading Map" << endl;

	mapLoad(yamlFilename,false,true, loadTracking);
	cout << "Map Loaded" << endl;

	std::vector<ORB_SLAM2::KeyFrame *> keyframes = system.mpMap->GetAllKeyFrames();

	ORB_SLAM2::KeyFrame *lastKf = *std::max_element(keyframes.begin(), keyframes.end(), [](const ORB_SLAM2::KeyFrame* a,const ORB_SLAM2::KeyFrame* b){return a->mnId < b->mnId;} );
	ORB_SLAM2::KeyFrame *secondLastKF = lastKf->GetParent();
	
	system.mpTracker->mpLastKeyFrame = lastKf;
	system.mpTracker->mnLastKeyFrameId = lastKf->mnId;

	system.mpTracker->mState = ORB_SLAM2::Tracking::OK;
	system.mpTracker->mnLastRelocFrameId = 0;
	system.mpTracker->mpReferenceKF = lastKf;
	
	{
		if (trackFrame)
		{
			system.mpTracker->mLastFrame = Frame(*trackFrame);
			system.mpTracker->mCurrentFrame = system.mpTracker->mLastFrame;
		}
		else 
		{
			ORB_SLAM2::Frame tmpf;
			tmpf.SetPose(lastKf->GetPose());
			tmpf.mpReferenceKF = secondLastKF;
			tmpf.N = lastKf->N;

			tmpf.mvpMapPoints =  std::vector<ORB_SLAM2::MapPoint* > (lastKf->mvpMapPoints);
			tmpf.mvDepth = vector<float>(lastKf->mvDepth);
			tmpf.mvuRight = vector<float>(lastKf->mvuRight);
			tmpf.mvKeys = std::vector<cv::KeyPoint>(lastKf->mvKeys.begin(),lastKf->mvKeys.end());    
			
			system.mpTracker->mLastFrame = tmpf;
			system.mpTracker->mCurrentFrame = tmpf;
			system.mpTracker->mVelocity = ORB_SLAM2::Mat();
		}
	}
	system.mpTracker->UpdateLocalKeyFrames();
	system.mpTracker->UpdateLocalPoints();
	system.mpLocalMapper->Release();
	system.mpViewer->Release();
    system.mpFrameDrawer->Update(system.mpTracker);


}


int Osmap::MapPointsSave(string filename)
{
	ofstream file;
	file.open(filename, std::ofstream::binary);

	// Serialize
	SerializedMappointArray serializedMappointArray;
	int nMP = serialize(vectorMapPoints, serializedMappointArray);

	// Closing
	if (!serializedMappointArray.SerializeToOstream(&file))
		// Signals the error
		nMP = -1;
	file.close();

	return nMP;
}

int Osmap::MapPointsLoad(string filename)
{
	ifstream file;
	file.open(filename, ifstream::binary);

	SerializedMappointArray serializedMappointArray;
	serializedMappointArray.ParseFromIstream(&file);
	int nMP = deserialize(serializedMappointArray, vectorMapPoints);
	cout << "Mappoints loaded: " << nMP << endl;

	file.close();
	return nMP;
}

int Osmap::KeyFramesSave(string filename)
{
	ofstream file;
	file.open(filename, std::ofstream::binary);

	// Serialize
	SerializedKeyframeArray serializedKeyFrameArray;
	int nKF = serialize(vectorKeyFrames, serializedKeyFrameArray);

	// Closing
	if (!serializedKeyFrameArray.SerializeToOstream(&file))
		// Signals the error
		nKF = -1;
	file.close();

	return nKF;
}

int Osmap::KeyFramesLoad(string filename)
{
	ifstream file;
	file.open(filename, ifstream::binary);
#ifndef OSMAP_DUMMY_MAP
	if (!currentFrame.mTcw.dims) // if map is no initialized, currentFrame has no pose, a pose is needed to create keyframes.
		currentFrame.mTcw = Mat::eye(4, 4, CV_32F);
#endif
	SerializedKeyframeArray serializedKeyFrameArray;
	serializedKeyFrameArray.ParseFromIstream(&file);
	int nKF = deserialize(serializedKeyFrameArray, vectorKeyFrames);

	deserialize_edges(serializedKeyFrameArray,vectorKeyFrames);

	cout << "Keyframes loaded: "
		 << nKF << endl;
	file.close();
	return nKF;
}

int Osmap::featuresSave(string filename)
{
	int nFeatures = 0;
	ofstream file;

	file.open(filename, ofstream::binary);
	if (
		options[FEATURES_FILE_DELIMITED] ||
		(!options[FEATURES_FILE_NOT_DELIMITED] && countFeatures() > FEATURES_MESSAGE_LIMIT))
	{
		// Saving with delimited ad hoc file format
		// Loop serializing blocks of no more than FEATURES_MESSAGE_LIMIT features, using Kendon Varda's function

		options.set(FEATURES_FILE_DELIMITED);

		// This Protocol Buffers stream must be deleted before closing file.  It happens automatically at }.
		::google::protobuf::io::OstreamOutputStream protocolbuffersStream(&file);
		vector<OsmapKeyFrame *> vectorBlock;
		vectorBlock.reserve(FEATURES_MESSAGE_LIMIT / 30);

		auto it = vectorKeyFrames.begin();
		while (it != vectorKeyFrames.end())
		{
			unsigned int n = (*it)->N;
			vectorBlock.clear();
			do
			{
				vectorBlock.push_back(*it);
				++it;
				if (it == vectorKeyFrames.end())
					break;
				KeyFrame *KF = *it;
				n += KF->N;
			} while (n <= FEATURES_MESSAGE_LIMIT);

			SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
			nFeatures += serialize(vectorBlock, serializedKeyframeFeaturesArray);
			writeDelimitedTo(serializedKeyframeFeaturesArray, &protocolbuffersStream);
		}
	}
	else
	{
		options.set(FEATURES_FILE_NOT_DELIMITED);
		SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
		nFeatures = serialize(vectorKeyFrames, serializedKeyframeFeaturesArray);
		if (!serializedKeyframeFeaturesArray.SerializeToOstream(&file))
		{
			cerr << "Error while serializing features file without delimitation." << endl;
			nFeatures = -1;
		}
	}
	file.close();

	return nFeatures;
}

int Osmap::featuresLoad(string filename)
{
	int nFeatures = 0;
	ifstream file;
	file.open(filename, ifstream::binary);
	auto *googleStream = new ::google::protobuf::io::IstreamInputStream(&file);
	SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
	if (options[FEATURES_FILE_DELIMITED])
	{
		while (true)
			if (readDelimitedFrom(googleStream, &serializedKeyframeFeaturesArray))
			{
				nFeatures += deserialize(serializedKeyframeFeaturesArray);
				cout << "Features deserialized in loop: "
					 << nFeatures << endl;
			}
			else
				break;
	}
	else
	{
		// Not delimited, pure Protocol Buffers
		serializedKeyframeFeaturesArray.ParseFromIstream(&file);
		nFeatures = deserialize(serializedKeyframeFeaturesArray);
	}
	cout << "Features loaded: " << nFeatures << endl;
	file.close();
	return nFeatures;
}

int Osmap::trackingFramesSave(string filename)
{
	ofstream file;
	file.open(filename, std::ofstream::binary);

	// Serialize
	SerializedTrackingFramesArray serializedTrackingFramesArray;
	int nFrames = serialize(vectorTrackingFrames, serializedTrackingFramesArray);

	// Closing
	if (!serializedTrackingFramesArray.SerializeToOstream(&file))
		// Signals the error
		nFrames = -1;
	file.close();
	return nFrames;
}



int Osmap::trackingFramesLoad(string filename)
{
	ifstream file;
	file.open(filename, ifstream::binary);

	SerializedTrackingFramesArray serializedTrackingFramesArray;
	serializedTrackingFramesArray.ParseFromIstream(&file);
	int nFrames = deserialize(serializedTrackingFramesArray);
	cout << "Frames loaded: " << nFrames << endl;

	file.close();
	return nFrames;
}




void Osmap::getMapPointsFromMap()
{
	vectorMapPoints.clear();
	vectorMapPoints.reserve(map.mspMapPoints.size());
	std::transform(map.mspMapPoints.begin(), map.mspMapPoints.end(), std::back_inserter(vectorMapPoints), [](MapPoint *pMP) -> OsmapMapPoint * { return static_cast<OsmapMapPoint *>(pMP); });
	sort(vectorMapPoints.begin(), vectorMapPoints.end(), [](const MapPoint *a, const MapPoint *b) { return a->mnId < b->mnId; });
}

void Osmap::setMapPointsToMap()
{
	map.mspMapPoints.clear();
	copy(vectorMapPoints.begin(), vectorMapPoints.end(), inserter(map.mspMapPoints, map.mspMapPoints.end()));
}

void Osmap::getKeyFramesFromMap()
{
	// Order keyframes by mnId
	vectorKeyFrames.clear();
	vectorKeyFrames.reserve(map.mspKeyFrames.size());
	std::transform(map.mspKeyFrames.begin(), map.mspKeyFrames.end(), std::back_inserter(vectorKeyFrames), [](KeyFrame *pKF) -> OsmapKeyFrame * { return static_cast<OsmapKeyFrame *>(pKF); });
	sort(vectorKeyFrames.begin(), vectorKeyFrames.end(), [](const KeyFrame *a, const KeyFrame *b) { return a->mnId < b->mnId; });
}

void Osmap::setKeyFramesToMap()
{
	map.mspKeyFrames.clear();
	copy(vectorKeyFrames.begin(), vectorKeyFrames.end(), inserter(map.mspKeyFrames, map.mspKeyFrames.end()));
}


void Osmap::setTrackingDataToTracker()
{
	list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

	for(auto frame_ptr : vectorTrackingFrames)
	{
		mlRelativeFramePoses.push_back(frame_ptr->relative_pose);
		mlpReferences.push_back(getKeyFrame(frame_ptr->ref_keyframe_id));
		mlFrameTimes.push_back(frame_ptr->time);
		mlbLost.push_back(frame_ptr->lost);
	}

	system.mpTracker->mlRelativeFramePoses = mlRelativeFramePoses; 
	system.mpTracker->mlpReferences = mlpReferences;
	system.mpTracker->mlFrameTimes = mlFrameTimes;
	system.mpTracker->mlbLost = mlbLost;

}

void Osmap::getTrackingDataFromTracker()
{
	const list<cv::Mat>& mlRelativeFramePoses = system.mpTracker->mlRelativeFramePoses;
    const list<KeyFrame*>& mlpReferences = system.mpTracker->mlpReferences;
    const list<double>& mlFrameTimes = system.mpTracker->mlFrameTimes;
    const list<bool>& mlbLost = system.mpTracker->mlbLost;

	auto pose_iter = mlRelativeFramePoses.begin();
	auto ref_frame_iter = mlpReferences.begin();
	auto time_iter = mlFrameTimes.begin();
	auto lost_iter = mlbLost.begin();

	size_t N = mlRelativeFramePoses.size();
	vectorTrackingFrames.clear();
	size_t ref_kf = 0U;

	for(size_t i = 0; i < N; i++)
	{
		auto frame_ptr = new OsmapTrackingFrame();
		
		//if (*lost_iter)
		ref_kf = (*ref_frame_iter)->mnId;
		//else 
		//	ref_kf = 0U;
		
		frame_ptr->relative_pose = *pose_iter;
		frame_ptr->ref_keyframe_id = ref_kf;
		frame_ptr->time = *time_iter; 
		frame_ptr->lost = *lost_iter;

		++pose_iter;
		++ref_frame_iter;
		++time_iter;
		++lost_iter;
		vectorTrackingFrames.push_back(frame_ptr);
	}
}


void Osmap::clearVectors()
{
	keyframeid2vectorkIdx.clear();
	vectorKeyFrames.clear();
	vectorMapPoints.clear();
	vectorK.clear();
}

void Osmap::parsePath(const string &path, string *filename, string *pathDirectory)
{
	size_t pos = path.find_last_of("\\/");
	if (std::string::npos == pos)
		// No directory separator, file is assumed.
		pos = 0;
	else
		// Last directory separator (/) will be in pathDirectory, not in filename.
		pos++;

	if (pathDirectory)
		*pathDirectory = path.substr(0, pos);
	if (filename)
		*filename = path.substr(pos);
	return;
}

void Osmap::depurate()
{
	// First erase MapPoint from KeyFrames, and then erase KeyFrames from MapPoints.

	// NULL out bad MapPoints in KeyFrame::mvpMapPoints
	for (auto pKF : map.mspKeyFrames)
	{
		// NULL out bad MapPoints and warns if not in map.  Usually doesn't find anything.
		auto pOKF = static_cast<OsmapKeyFrame *>(pKF);
		auto &pMPs = pOKF->mvpMapPoints;
		for (int i = pMPs.size(); --i >= 0;)
		{
			auto pOMP = static_cast<OsmapMapPoint *>(pMPs[i]);

			if (!pOMP)
				continue; // Ignore if NULL

			if (pOMP->mbBad)
			{
				// If MapPoint is bad, NULL it in keyframe's observations.
				cerr << "depurate(): Nullifying bad MapPoint " << pOMP->mnId << " in KeyFrame " << pOKF->mnId << endl;
				pMPs[i] = NULL;
			}
			else if (!map.mspMapPoints.count(pOMP) && !options[NO_APPEND_FOUND_MAPPOINTS])
			{
				// If MapPoint is not in map, append it to the map
				map.mspMapPoints.insert(pOMP);
				cout << "depurate(): APPEND_FOUND_MAPPOINTS: MapPoint " << pOMP->mnId << " added to map. ";
			}
		}
	}
}

void Osmap::rebuild(bool noSetBad)
{
	/*
	 * On every KeyFrame:
	 * - Builds the map database
	 * - UpdateConnections to rebuild covisibility graph
	 * - MapPoint::AddObservation on each point to rebuild MapPoint:mObservations y MapPoint:mObs
	 */
	cout << "Rebuilding map:" << endl;
	keyFrameDatabase.clear();

	if (noSetBad)
		options.set(NO_SET_BAD);

	log("Processing", vectorKeyFrames.size(), "keyframes");
	for (auto *pKF : vectorKeyFrames)
	{
		LOGV(pKF);
		LOGV(pKF->mnId);

		pKF->mbNotErase = !pKF->mspLoopEdges.empty();
		LOGV(pKF->mbNotErase);

		// Build BoW vectors and the inverse feature index
		// If the BowVec is already populated, we loaded a BoW from the file
		// Else, BowVecs wasn't saved.
		if (pKF->mBowVec.size())
		{
			// TODO: Avoid computing the BoW vector
			auto backup=pKF->mBowVec;
			pKF->ComputeBoW();
			pKF->mBowVec = backup;
		}	
		else
			pKF->ComputeBoW();
		
		log("BoW computed");

		// Build many pose matrices
		pKF->SetPose(pKF->Tcw);
		LOGV(pKF->Tcw);
		log("Pose set");

		/*
		 * Rebuilding grid.
		 * Code from Frame::AssignFeaturesToGrid()
		 */
		std::vector<std::size_t> grid[pKF->mnGridCols][pKF->mnGridRows];
		int nReserve = 0.5f * pKF->N / (pKF->mnGridCols * pKF->mnGridRows);
		for (int i = 0; i < pKF->mnGridCols; i++)
			for (int j = 0; j < pKF->mnGridRows; j++)
				grid[i][j].reserve(nReserve);
		log("Grid built");

		for (int i = 0; i < pKF->N; i++)
		{
			const cv::KeyPoint &kp = pKF->mvKeysUn[i];
			int posX = round((kp.pt.x - pKF->mnMinX) * pKF->mfGridElementWidthInv);
			int posY = round((kp.pt.y - pKF->mnMinY) * pKF->mfGridElementHeightInv);

			//Keypoint's coordinates are undistorted, which could cause to go out of the image
			if (!(posX < 0 || posX >= pKF->mnGridCols || posY < 0 || posY >= pKF->mnGridRows))
				grid[posX][posY].push_back(i);
		}
		log("Grid full");

		pKF->mGrid.resize(pKF->mnGridCols);
		for (int i = 0; i < pKF->mnGridCols; i++)
		{
			pKF->mGrid[i].resize(pKF->mnGridRows);
			for (int j = 0; j < pKF->mnGridRows; j++)
				pKF->mGrid[i][j] = grid[i][j];
		}
		log("Grid fitted");

		// Append keyframe to the database
		keyFrameDatabase.add(pKF);
		log("Added Database");

		// Rebuild MapPoints obvervations
		size_t n = pKF->mvpMapPoints.size();
		for (size_t i = 0; i < n; i++)
		{
			MapPoint *pMP = pKF->mvpMapPoints[i];
			if (pMP)
				pMP->AddObservation(pKF, i);
		}
		log("Observations rebuilt");

		// Calling UpdateConnections in mnId order rebuilds the covisibility graph and the spanning tree.
		pKF->UpdateConnections();
	}

	// Last KeyFrame's id
	map.mnMaxKFid = vectorKeyFrames.back()->mnId;

	// Next KeyFrame id
	KeyFrame::nNextId = map.mnMaxKFid + 1;




	/*
		Rather than the old Osmap behavior of deleting keyframes without parents, 
		instead find the first valid parent candidate older than this frame 
		and make it this frame's parent. 
	*/
	// Retry on isolated keyframes
	//KeyFrame * origin = vectorKeyFrames.front();

	log("Heuristic Parent Edges");
	size_t assigned = 0;
	for(size_t i = 1, N = vectorKeyFrames.size(); i < N;i++)
	{
		KeyFrame* pKF = vectorKeyFrames[i];
		
		if (!pKF->mpParent)
		{
			pKF->ChangeParent(vectorKeyFrames[i-1]);
			pKF->mbFirstConnection = false;
			assigned++;
		}
		pKF->UpdateConnections();
		/*std::cout << pKF->mnId << std::endl;
		for(size_t i=0;i<pKF->mvpOrderedConnectedKeyFrames.size();i++)
		{

			std::cout << pKF->mvpOrderedConnectedKeyFrames[i]->mnId << "(" <<  pKF->mvOrderedWeights[i] <<") ";
		}
		std::cout << std::endl;
		*/

	}
	
		


		/*std::cout << "Edges " << pKF->mnId;
		
		for(auto kf : pKF->mvpOrderedConnectedKeyFrames)
		{
			std::cout << kf->mnId << ' ';
		}
		std::cout << std::endl;*/

		
	

	
	/*log("Scanning for Disconnected Frames");

	set<KeyFrame*> floating_frames(vectorKeyFrames.begin(),vectorKeyFrames.end());
	list<KeyFrame*> traverse({origin});
	while(!traverse.empty())
	{
		KeyFrame* front = traverse.front();
		traverse.pop_front();
		floating_frames.erase(front);

		for(KeyFrame* child: front->GetChilds())
		{
			if(child)
				traverse.push_back(child);
		}
		
	}

	cout << "DISCONNECTED FRAMES" << endl;
	for(KeyFrame* frame : floating_frames)
	{
		cout << frame->mnId << ' ';
	}
	cout << endl;
	
	*/
	/*
	 * Check and fix the spanning tree created with UpdateConnections.
	 * Rebuilds the spanning tree asigning a mpParent to every orphan KeyFrame without, except that with id 0.
 	 * It ends when every KeyFrame has a parent.
	 */

	// mvpKeyFrameOrigins should be empty at this point, and must contain only one element, the first keyframe.
	map.mvpKeyFrameOrigins.clear();
	map.mvpKeyFrameOrigins.push_back(*vectorKeyFrames.begin());

	// Number of parents assigned in each iteration and in total.  Usually 0.
	/*int nParents = -1, nParentsTotal = 0;
	log("Rebuilding spanning tree.");
	while (nParents)
	{
		nParents = 0;
		for (auto pKF : vectorKeyFrames)
			if (!pKF->mpParent && pKF->mnId) // Process all keyframes without parent, exccept id 0
				for (auto *pConnectedKF : pKF->mvpOrderedConnectedKeyFrames)
				{
					auto poConnectedKF = static_cast<OsmapKeyFrame *>(pConnectedKF);
					if (poConnectedKF->mpParent || poConnectedKF->mnId == 0)
					{ // Parent found: not orphan or id 0
						nParents++;
						pKF->ChangeParent(pConnectedKF);
						break;
					}
				}
		nParentsTotal += nParents;
		log("Parents assigned in this loop:", nParents);
	}
	log("Parents assigned in total:", nParentsTotal);
	*/
	/*
	 * On every MapPoint:
	 * - Rebuilds mpRefKF as the first observation, which should be the KeyFrame with the lowest id
	 * - Rebuilds many properties with UpdateNormalAndDepth()
	 */
	log("Processing", vectorMapPoints.size(), "mappoints.");
	for (OsmapMapPoint *pMP : vectorMapPoints)
	{
		LOGV(pMP)
		LOGV(pMP->mnId)
		// Rebuilds mpRefKF.  Requires mObservations. 
		if (pMP->mObservations.empty()) // !options[NO_SET_BAD] &&
		{
			/*cerr << "MP " << pMP->mnId << " without observations."
				 << "  Set bad." << endl;*/
			// TODO: Workaround for mappoint with id of 0 
			pMP->SetBadFlag();
			continue;
		}

		// Asumes the first observation in mappoint has the lowest mnId.  Processed keyframes in mnId order ensures this.
		auto pair = (*pMP->mObservations.begin());
		pMP->mpRefKF = pair.first;

		/* UpdateNormalAndDepth() requires prior rebuilding of mpRefKF, and rebuilds:
		 * - mNormalVector
		 * - mfMinDistance
		 * - mfMaxDistance
		 */
		pMP->UpdateNormalAndDepth();
	}
	MapPoint::nNextId = vectorMapPoints.back()->mnId + 1;
}

void Osmap::getVectorKFromKeyframes()
{
	vectorK.clear();
	keyframeid2vectorkIdx.resize(KeyFrame::nNextId);					 // Assume map is not ill formed so nNextId is ok, thus no keyframe's id is bigger than this.
	fill(keyframeid2vectorkIdx.begin(), keyframeid2vectorkIdx.end(), 0); // Fill with index 0 to prevent segfault from unknown bugs.

	if (vectorKeyFrames.empty())
		getKeyFramesFromMap();

	//for(auto &pKF:map.mspKeyFrames){
	for (auto pKF : vectorKeyFrames)
	{
		// Test if K can be found in vectorK.  If new, add it to the end of vectorK.
		//Mat &K = const_cast<cv::Mat &> (pKF->mK);
		const Mat &K = pKF->mK;

		// Will be the index of K in vectorK
		unsigned int i;
		for (i = 0; i < vectorK.size(); i++)
		{
			const Mat &vK = *vectorK[i];

			// Tests: break if found

			// Quick test
			if (K.data == vK.data)
				break;

				// Slow test, compare each element
/*
      if(
        K.at<float>(0,0) == vK.at<float>(0,0) &&
        K.at<float>(1,1) == vK.at<float>(1,1) &&
        K.at<float>(0,2) == vK.at<float>(0,2) &&
        K.at<float>(1,2) == vK.at<float>(1,2)
      ) break;
*/
#define DELTA 0.1
			if (
				abs(K.at<float>(0, 0) - vK.at<float>(0, 0)) < DELTA &&
				abs(K.at<float>(1, 1) - vK.at<float>(1, 1)) < DELTA &&
				abs(K.at<float>(0, 2) - vK.at<float>(0, 2)) < DELTA &&
				abs(K.at<float>(1, 2) - vK.at<float>(1, 2)) < DELTA)
				break;
		}

		// if not found, push
		if (i >= vectorK.size())
		{
			// add new K
			vectorK.push_back(&K);
			vectorBaselines.push_back(pKF->mb);

		}

		// i is the vectorK index for this keyframe
		keyframeid2vectorkIdx[pKF->mnId] = i;
	}
}

int Osmap::countFeatures()
{
	int n = 0;
	for (auto pKP : vectorKeyFrames)
		n += pKP->N;

	return n;
}

// Utilities
MapPoint *Osmap::getMapPoint(unsigned int id)
{
	//for(auto pMP : map.mspMapPoints)
	for (auto pMP : vectorMapPoints)
		if (pMP->mnId == id)
			return pMP;

	// Not found
	return NULL;
}

OsmapKeyFrame *Osmap::getKeyFrame(unsigned int id)
{
	//for(auto it = map.mspKeyFrames.begin(); it != map.mspKeyFrames.end(); ++it)
	for (auto pKF : vectorKeyFrames)
		if (pKF->mnId == id)
			return pKF;

	// If not found
	return NULL;
}

// K matrix ================================================================================================
void Osmap::serialize(const Mat &k, const float &b, SerializedK *serializedK)
{
	serializedK->set_fx(k.at<float>(0, 0));
	serializedK->set_fy(k.at<float>(1, 1));
	serializedK->set_cx(k.at<float>(0, 2));
	serializedK->set_cy(k.at<float>(1, 2));
	serializedK->set_baseline(b);
}

void Osmap::deserialize(const SerializedK &serializedK, Mat &m, float b)
{
	m = Mat::eye(3, 3, CV_32F);
	m.at<float>(0, 0) = serializedK.fx();
	m.at<float>(1, 1) = serializedK.fy();
	m.at<float>(0, 2) = serializedK.cx();
	m.at<float>(1, 2) = serializedK.cy();
	b = serializedK.baseline();
}

//void Osmap::serialize(const vector<Mat *> &vK, SerializedKArray &serializedKArray)
//{
//}

//void Osmap::deserialize(const SerializedKArray &serializedKArray, vector<Mat *> &vK)
//{
//}

// Descriptor ================================================================================================
void Osmap::serialize(const Mat &m, SerializedDescriptor *serializedDescriptor)
{
	assert(m.rows == 1 && m.cols == 32);
	for (unsigned int i = 0; i < 8; i++)
		serializedDescriptor->add_block(((unsigned int *)m.data)[i]);
}

void Osmap::deserialize(const SerializedDescriptor &serializedDescriptor, Mat &m)
{
	assert(serializedDescriptor.block_size() == 8);
	m = Mat(1, 32, CV_8UC1);
	for (unsigned int i = 0; i < 8; i++)
		((unsigned int *)m.data)[i] = serializedDescriptor.block(i);
}

// Pose ================================================================================================
void Osmap::serialize(const Mat &m, SerializedPose *serializedPose)
{
	float *pElement = (float *)m.data;
	for (unsigned int i = 0; i < 12; i++)
		serializedPose->add_element(pElement[i]);
}

void Osmap::deserialize(const SerializedPose &serializedPose, Mat &m)
{
	assert(serializedPose.element_size() == 12);
	m = Mat::eye(4, 4, CV_32F);
	float *pElement = (float *)m.data;
	for (unsigned int i = 0; i < 12; i++)
		pElement[i] = serializedPose.element(i);
}

// Position ================================================================================================
void Osmap::serialize(const Mat &m, SerializedPosition *serializedPosition)
{
	serializedPosition->set_x(m.at<float>(0, 0));
	serializedPosition->set_y(m.at<float>(1, 0));
	serializedPosition->set_z(m.at<float>(2, 0));
}

void Osmap::deserialize(const SerializedPosition &serializedPosition, Mat &m)
{
	m = Mat(3, 1, CV_32F);
	m.at<float>(0, 0) = serializedPosition.x();
	m.at<float>(1, 0) = serializedPosition.y();
	m.at<float>(2, 0) = serializedPosition.z();
}

// KeyPoint ================================================================================================
void Osmap::serialize(const KeyPoint &kp, SerializedKeypoint *serializedKeypoint)
{
	serializedKeypoint->set_ptx(kp.pt.x);
	serializedKeypoint->set_pty(kp.pt.y);
	serializedKeypoint->set_octave(kp.octave);
	serializedKeypoint->set_angle(kp.angle);
}

void Osmap::deserialize(const SerializedKeypoint &serializedKeypoint, KeyPoint &kp)
{
	kp.pt.x = serializedKeypoint.ptx();
	kp.pt.y = serializedKeypoint.pty();
	kp.octave = serializedKeypoint.octave();
	kp.angle = serializedKeypoint.angle();
}

// MapPoint ================================================================================================
void Osmap::serialize(const OsmapMapPoint &mappoint, SerializedMappoint *serializedMappoint)
{
	serializedMappoint->set_id(mappoint.mnId);
	serialize(mappoint.mWorldPos, serializedMappoint->mutable_position());
	serializedMappoint->set_visible(mappoint.mnVisible);
	serializedMappoint->set_found(mappoint.mnFound);
	//if(options[NO_FEATURES_DESCRIPTORS])	// This is the only descriptor to serialize	** This line is disable to force mappoint descriptor serialization, while it's not being reconstructed in rebuild. **
	serialize(mappoint.mDescriptor, serializedMappoint->mutable_briefdescriptor());
}

OsmapMapPoint *Osmap::deserialize(const SerializedMappoint &serializedMappoint)
{
	OsmapMapPoint *pMappoint = new OsmapMapPoint(this);
	pMappoint->mnId = serializedMappoint.id();
	pMappoint->mnVisible = serializedMappoint.visible();
	pMappoint->mnFound = serializedMappoint.found();
	if (serializedMappoint.has_briefdescriptor())
		deserialize(serializedMappoint.briefdescriptor(), pMappoint->mDescriptor);
	if (serializedMappoint.has_position())
		deserialize(serializedMappoint.position(), pMappoint->mWorldPos);

	return pMappoint;
}

int Osmap::serialize(const vector<OsmapMapPoint *> &vectorMP, SerializedMappointArray &serializedMappointArray)
{
	for (auto pMP : vectorMP)
		serialize(*pMP, serializedMappointArray.add_mappoint());

	return vectorMP.size();
}

int Osmap::deserialize(const SerializedMappointArray &serializedMappointArray, vector<OsmapMapPoint *> &vectorMapPoints)
{
	int i, n = serializedMappointArray.mappoint_size();
	for (i = 0; i < n; i++)
		vectorMapPoints.push_back(deserialize(serializedMappointArray.mappoint(i)));

	return i;
}

// KeyFrame ================================================================================================
void Osmap::serialize(const OsmapKeyFrame &keyframe, SerializedKeyframe *serializedKeyframe)
{
	serializedKeyframe->set_id(keyframe.mnId);
	serialize(keyframe.Tcw, serializedKeyframe->mutable_pose());
	serializedKeyframe->set_timestamp(keyframe.mTimeStamp);
	if (options[K_IN_KEYFRAME])
		serialize(keyframe.mK, keyframe.mb, serializedKeyframe->mutable_kmatrix());
	else
		serializedKeyframe->set_kindex(keyframeid2vectorkIdx[keyframe.mnId]);
	if (!keyframe.mspLoopEdges.empty())
		for (auto loopKF : keyframe.mspLoopEdges)
			// Only serialize id of keyframes already serialized, to easy deserialization.
			if (keyframe.mnId > loopKF->mnId)
				serializedKeyframe->add_loopedgesids(loopKF->mnId);
	if (keyframe.mBowVec.size())
	{
		auto serialBoW = serializedKeyframe->mutable_bow();
		for (std::pair<uint, double> wordpair : keyframe.mBowVec)
			 serialBoW->operator[](wordpair.first) = wordpair.second;
	}
	if(keyframe.mvMargEdges.size())
	{
		serialize(keyframe.mvMargEdges,*serializedKeyframe);
	}
}

OsmapKeyFrame *Osmap::deserialize(const SerializedKeyframe &serializedKeyframe)
{
	OsmapKeyFrame *pKeyframe = new OsmapKeyFrame(this);

	pKeyframe->mnId = serializedKeyframe.id();
	const_cast<double &>(pKeyframe->mTimeStamp) = serializedKeyframe.timestamp();

	if (serializedKeyframe.has_pose())
		deserialize(serializedKeyframe.pose(), pKeyframe->Tcw);

	if (serializedKeyframe.has_kmatrix())
		// serialized with K_IN_KEYFRAME option, doesn't use K list in yaml
		deserialize(serializedKeyframe.kmatrix(), const_cast<cv::Mat &>(pKeyframe->mK), const_cast<float &>(pKeyframe->mb));
	else
	{
		// serialized with default no K_IN_KEYFRAME option, K list in yaml
		const_cast<cv::Mat &>(pKeyframe->mK) = *vectorK[serializedKeyframe.kindex()];
		pKeyframe->mb = vectorBaselines[serializedKeyframe.kindex()];
	}



	// fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
	/*
	const_cast<float&>(pKeyframe->fx) = pKeyframe->mK.at<float>(0,0);
	const_cast<float&>(pKeyframe->fy) = pKeyframe->mK.at<float>(1,1);
	const_cast<float&>(pKeyframe->invfx) = 1.0f / pKeyframe->invfx;
	const_cast<float&>(pKeyframe->invfy) = 1.0f / pKeyframe->invfy;
	const_cast<float&>(pKeyframe->cx) = pKeyframe->mK.at<float>(0,2);
	const_cast<float&>(pKeyframe->cy) = pKeyframe->mK.at<float>(1,2); 
	*/
	pKeyframe->fx = pKeyframe->mK.at<float>(0, 0);
	pKeyframe->fy = pKeyframe->mK.at<float>(1, 1);
	pKeyframe->invfx = 1.0f / pKeyframe->invfx;
	pKeyframe->invfy = 1.0f / pKeyframe->invfy;
	pKeyframe->cx = pKeyframe->mK.at<float>(0, 2);
	pKeyframe->cy = pKeyframe->mK.at<float>(1, 2);
	pKeyframe->mbf = pKeyframe->mb * pKeyframe->fx;
	pKeyframe->loaded = true;

	if (serializedKeyframe.loopedgesids_size())
	{
		// Only ids of keyframes already deserialized and present on vectorKeyFrames
		for (int i = 0; i < serializedKeyframe.loopedgesids_size(); i++)
		{
			unsigned int loopEdgeId = serializedKeyframe.loopedgesids(i);
			OsmapKeyFrame *loopEdgeKF = getKeyFrame(loopEdgeId);
			loopEdgeKF->mspLoopEdges.insert(pKeyframe);
			pKeyframe->mspLoopEdges.insert(loopEdgeKF);
		}
	}

	if (serializedKeyframe.bow_size())
	{
		
		for (auto wordpair : serializedKeyframe.bow())
		{
			pKeyframe->mBowVec.addWeight(wordpair.first,wordpair.second);
		}

	}


	return pKeyframe;
}




int Osmap::serialize(const vector<OsmapKeyFrame *> &vectorKF, SerializedKeyframeArray &serializedKeyframeArray)
{
	for (auto pKF : vectorKF)
		serialize(*pKF, serializedKeyframeArray.add_keyframe());

	return vectorKF.size();
}

int Osmap::deserialize(const SerializedKeyframeArray &serializedKeyframeArray, vector<OsmapKeyFrame *> &vectorKeyFrames)
{
	int i, n = serializedKeyframeArray.keyframe_size();
	for (i = 0; i < n; i++)
		vectorKeyFrames.push_back(deserialize(serializedKeyframeArray.keyframe(i)));

	return i;
}

// Feature ================================================================================================
void Osmap::serialize(const OsmapKeyFrame &keyframe, SerializedKeyframeFeatures *serializedKeyframeFeatures)
{
	serializedKeyframeFeatures->set_keyframe_id(keyframe.mnId);
	for (int i = 0; i < keyframe.N; i++)
	{
		if (!options[ONLY_MAPPOINTS_FEATURES] || keyframe.mvpMapPoints[i])
		{ // If chosen to only save mappoints features, check if there is a mappoint.
			SerializedFeature &serializedFeature = *serializedKeyframeFeatures->add_feature();

			// KeyPoint
			serialize(keyframe.mvKeysUn[i], serializedFeature.mutable_keypoint());

			// If there is a MapPoint, serialize it
			if (keyframe.mvpMapPoints[i])
				serializedFeature.set_mappoint_id(keyframe.mvpMapPoints[i]->mnId);

			// Serialize descriptor but skip if chosen to not do so.
			if (!options[NO_FEATURES_DESCRIPTORS]) //
				serialize(keyframe.mDescriptors.row(i), serializedFeature.mutable_briefdescriptor());

			if (keyframe.mvuRight[i] > 0)
				serializedFeature.set_uright(keyframe.mvuRight[i]);
		}
	}
}




void Osmap::deserialize(const SerializedFeature& feature, MapPoint** mappoint, cv::KeyPoint &keypoint, cv::Mat& descriptors, size_t index, float& right, float& depth, float bf)
{
	if (feature.mappoint_id())
	{
		*mappoint =  getMapPoint(feature.mappoint_id());
	}
	else 
		*mappoint = nullptr;

	if (feature.has_keypoint())
	{
		deserialize(feature.keypoint(), keypoint);
		if (feature.uright() > 0 && bf > 0)
		{
			right = feature.uright();
			depth = bf / (feature.keypoint().ptx() - feature.uright());
		}
		else 
		{
			right = -1.0f;
			depth = -1.0f;
		}
	}
	if (feature.has_briefdescriptor())
	{
		Mat tmp;
		deserialize(feature.briefdescriptor(), tmp);
		tmp.copyTo(descriptors.row(index));
	}
		
}

OsmapKeyFrame *Osmap::deserialize(const SerializedKeyframeFeatures &serializedKeyframeFeatures)
{
	/*
	unsigned int KFid = serializedKeyframeFeatures.keyframe_id();
	OsmapKeyFrame *pKF = getKeyFrame(KFid);
	if (pKF)
	{
		size_t n = (size_t) serializedKeyframeFeatures.feature_size();
		const_cast<int &>(pKF->N) = n;

		std::vector<MapPoint*>& mappoints = pKF->mvpMapPoints;
		//const_cast<std::vector<cv::KeyPoint> &>(pKF->mvKeysUn).resize(n);

		std::vector<cv::KeyPoint>& keypoints = const_cast<std::vector<cv::KeyPoint> &>(pKF->mvKeysUn);
		std::vector<float>& right = const_cast<std::vector<float> &>(pKF->mvuRight);
		std::vector<float>& depth = const_cast<std::vector<float> &>(pKF->mvDepth);
		cv::Mat& descriptors = const_cast<cv::Mat &>(pKF->mDescriptors);

		descriptors = Mat(n, 32, CV_8UC1);
		right = std::vector<float>(n,-1.0f);
		depth = std::vector<float>(n,-1.0f);
		mappoints = std::vector<MapPoint*>(n,nullptr);
		keypoints.resize(n);


		for(int i=0;i<n;i++)
		{
 			deserialize(serializedKeyframeFeatures.feature(i),&mappoints[i],keypoints[i],descriptors,(size_t) i,right[i],depth[i],pKF->mbf);
		}

	}
	*/
	
	unsigned int KFid = serializedKeyframeFeatures.keyframe_id();
	size_t empty_descriptors = 0;
	size_t mappoint_descriptors = 0;
	OsmapKeyFrame *pKF = getKeyFrame(KFid);
	if (pKF)
	{
		int n = serializedKeyframeFeatures.feature_size();
		const_cast<int &>(pKF->N) = n;
		const_cast<std::vector<cv::KeyPoint> &>(pKF->mvKeysUn).resize(n);
		pKF->mvpMapPoints.resize(n);
		const_cast<cv::Mat &>(pKF->mDescriptors) = Mat(n, 32, CV_8UC1); // n descriptors

		vector<float> vRight(n, -1.0f);
		vector<float> vDepth(n, -1.0f);

		for (int i = 0; i < n; i++)
		{
			const SerializedFeature &feature = serializedKeyframeFeatures.feature(i);
			ORB_SLAM2::MapPoint *mappoint = nullptr;

			if (feature.mappoint_id())
			{
				mappoint =  getMapPoint(feature.mappoint_id());
				pKF->mvpMapPoints[i] =mappoint;
			}
			if (feature.has_keypoint())
				deserialize(feature.keypoint(), const_cast<cv::KeyPoint &>(pKF->mvKeysUn[i]));
			if (feature.has_briefdescriptor())
			{
				Mat descriptor;
				deserialize(feature.briefdescriptor(), descriptor);
				descriptor.copyTo(pKF->mDescriptors.row(i));
			}
			else
			{
				//if (mappoint)
				//{
				//	auto desc = mappoint->GetDescriptor();
				//	if(desc.size)
				//		{
				//		desc.copyTo(pKF->mDescriptors.row(i));
				//		mappoint_descriptors++;
				//	}
				//}
				//else 
				//{
				//	Mat descriptor = Mat::zeros(1, 32, CV_8UC1);
				//		descriptor.copyTo(pKF->mDescriptors.row(i));
				//	empty_descriptors++;
				//}
			}

			if (feature.uright() > 0)
			{
				if (pKF->mbf > 0)
				{
					vRight[i] = feature.uright();
					vDepth[i] = pKF->mbf / (feature.keypoint().ptx() - feature.uright());
				}
				else 
					cerr << "Keyframe" << KFid << " provides no baseline. Feature nr " << i << " stereo info disregarded." << endl;
			}
		}

		const_cast<std::vector<cv::KeyPoint> &>(pKF->mvKeys) = pKF->mvKeysUn;
		const_cast<std::vector<float> &>(pKF->mvuRight) = vRight;
		const_cast<std::vector<float> &>(pKF->mvDepth) = vDepth;
		
	}
	else
	{
		cerr << "KeyFrame id " << KFid << "not found while deserializing features: skipped.  Inconsistence between keyframes and features serialization files." << endl;
	}
	if (empty_descriptors || mappoint_descriptors)
	{
		cerr << "KeyFrame" << KFid << " had empty descriptors." << mappoint_descriptors << " descriptors loaded from mappoints. " << empty_descriptors << " empty descriptors." << endl;
	}
	
	return pKF;
}

int Osmap::serialize(const vector<OsmapKeyFrame *> &vectorKF, SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray)
{
	unsigned int nFeatures = 0;
	for (auto pKF : vectorKF)
	{
		serialize(*pKF, serializedKeyframeFeaturesArray.add_feature());
		nFeatures += pKF->N;
	}

	return nFeatures;
}

int Osmap::deserialize(const SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray)
{
	int nFeatures = 0, i, n = serializedKeyframeFeaturesArray.feature_size();
	for (i = 0; i < n; i++)
	{
		KeyFrame *pKF = deserialize(serializedKeyframeFeaturesArray.feature(i));
		if (pKF)
			nFeatures += pKF->N;
	}

	return nFeatures;
}
// TrackingFrame ================================================================================================

void Osmap::serialize(const OsmapTrackingFrame& frame, SerializedTrackingFrame* serializedFrame)
{
	serialize(frame.relative_pose, serializedFrame->mutable_pose());
	serializedFrame->set_frame_time(frame.time);
	serializedFrame->set_lost(frame.lost);
	serializedFrame->set_ref_keyframe(frame.ref_keyframe_id);
}


int Osmap::serialize(const vector<OsmapTrackingFrame*>& vectorFrames, SerializedTrackingFramesArray& serializedTrackingFramesArray)
{
	int nFrames = 0;
	for(auto frame : vectorFrames)
	{
		serialize(*frame, serializedTrackingFramesArray.add_frame());
		nFrames++;
	}


	Frame& frame = system.mpTracker->mCurrentFrame;
	for (int i = 0; i < frame.N; i++)
	{
	
		SerializedFeature *serializedFeature =  serializedTrackingFramesArray.add_track_feature();

		// KeyPoint
		serialize(frame.mvKeysUn[i], serializedFeature->mutable_keypoint());

		// If there is a MapPoint, serialize it
		if (frame.mvpMapPoints[i])
			serializedFeature->set_mappoint_id(frame.mvpMapPoints[i]->mnId);

		// Serialize descriptor but skip if chosen to not do so.
		serialize(frame.mDescriptors.row(i), serializedFeature->mutable_briefdescriptor());

		if (frame.mvuRight[i] > 0)
			serializedFeature->set_uright(frame.mvuRight[i]);
	}

	return nFrames;
}

OsmapTrackingFrame* Osmap::deserialize(const SerializedTrackingFrame& serializedTrackingFrame)
{
	OsmapTrackingFrame * frame = new OsmapTrackingFrame();
	deserialize(serializedTrackingFrame.pose(), frame->relative_pose);
	frame->ref_keyframe_id = serializedTrackingFrame.ref_keyframe();
	frame->time = serializedTrackingFrame.frame_time();
	frame->lost = serializedTrackingFrame.lost();
	return frame;
}


int Osmap::deserialize(const SerializedTrackingFramesArray& serializedTrackingFramesArray)
{
	int nFrames = 0;
	{  
		vectorTrackingFrames.clear();
		int n = serializedTrackingFramesArray.frame_size();
		for(int i = 0; i < n; i++)
		{
			OsmapTrackingFrame *frame = deserialize(serializedTrackingFramesArray.frame(i));
			if (frame)
			{
				vectorTrackingFrames.push_back(frame);
				nFrames++;
			}
		}
	}
	
	//if (false)
	if (serializedTrackingFramesArray.track_feature_size())
	{
		std::cout << "Tracking Features " << serializedTrackingFramesArray.track_feature_size() << std::endl;
		int n = serializedTrackingFramesArray.track_feature_size();
		trackFrame = new Frame(currentFrame);


		const OsmapTrackingFrame& last_track = *vectorTrackingFrames.back();
		KeyFrame* ref_keyframe = getKeyFrame(last_track.ref_keyframe_id);
		cv::Mat Tcw = last_track.relative_pose*ref_keyframe->GetPose();

		cv::Mat Tcw2 = vectorTrackingFrames[vectorTrackingFrames.size()-2]->relative_pose*ref_keyframe->GetPose();
		cv::Mat velocity =  Tcw*Tcw2.inv();
		system.mpTracker->mVelocity = velocity;
		

		trackFrame->fx = ref_keyframe->fx;
		trackFrame->fy = ref_keyframe->fy;
		trackFrame->invfx = ref_keyframe->invfx;
		trackFrame->invfy = ref_keyframe->invfy;
		trackFrame->cx = ref_keyframe->cx;
		trackFrame->cy = ref_keyframe->cy;
		trackFrame->mbf = ref_keyframe->mbf;
		trackFrame->mb = ref_keyframe->mb;


		
		trackFrame->SetPose(Tcw);
		trackFrame->mvpMapPoints = std::vector<MapPoint*>(n,nullptr);
		trackFrame->mvKeysUn.resize(n);
		trackFrame->mDescriptors  = Mat(n, 32, CV_8UC1);
		trackFrame->mvuRight = std::vector<float>(n,-1.0f);
		trackFrame->mvDepth = std::vector<float>(n,-1.0f);
		trackFrame->mvbOutlier = std::vector<bool>(n,false);
		trackFrame->mpReferenceKF = ref_keyframe;
		trackFrame->mnId = nFrames-1;
		trackFrame->nNextId = nFrames;

		for(int i = 0; i < n; i++)
			deserialize(serializedTrackingFramesArray.track_feature(i),&trackFrame->mvpMapPoints[i],trackFrame->mvKeysUn[i],trackFrame->mDescriptors,(size_t) i,trackFrame->mvuRight[i],trackFrame->mvDepth[i],trackFrame->mbf);
		
		trackFrame->AssignFeaturesToGrid();
		trackFrame->ComputeBoW();

	}
	else 
		trackFrame = nullptr;

	return nFrames;
}




// MargEdge ================================================================================================
void Osmap::serialize(const KeyFrame::MargEdge &edge, SerializedPoseEdge *serializedEdge)
{
	for(int i=0; i < 6;i++)
		for(int j=0;j<6;j++)
			if (i>=j)
				serializedEdge->add_info(edge.info.at<double>(i,j));
	for(int i=0;i < 6;i++)
	{
		serializedEdge->add_measurement(edge.measurement.at<double>(i,0));
	}
	serializedEdge->set_ref_id(edge.ref_kf->mnId);
	serializedEdge->set_cnt(edge.cnt);
}

void Osmap::deserialize(const SerializedPoseEdge &serializedEdge, KeyFrame::MargEdge &edge)
{
	KeyFrame* ref_kf = getKeyFrame(serializedEdge.ref_id());
	assert(ref_kf !=  NULL);
	assert(serializedEdge.info_size() == 21);

	edge.info = Mat::zeros(6,6,CV_64F);
	int k = 0;
	for(int i=0;i<6;i++)
		for(int j=i;j<6;j++)
		{
			double val = serializedEdge.info(k++);
			edge.info.at<double>(i,j) = val;
			if (i != j)
				edge.info.at<double>(j,i) = val;
		}

	edge.measurement = Mat::zeros(6,1,CV_64F);
	for(int i=0;i<6;i++)
	{
		edge.measurement.at<double>(i,0)=serializedEdge.measurement(i);
	}

	edge.cnt = serializedEdge.cnt();
	edge.ref_kf = ref_kf ;
}

void Osmap::serialize(const std::vector<KeyFrame::MargEdge> &edges, SerializedKeyframe &keyframe)
{
	for(size_t i=0;i<edges.size();i++ )
	{
		SerializedPoseEdge* new_edge = keyframe.add_marg_edges();
		serialize(edges[i],new_edge);
	}
}

void Osmap::deserialize(const SerializedKeyframe &keyframe, std::vector<KeyFrame::MargEdge> &edges)
{
	for(int i=0;i<keyframe.marg_edges_size();i++)
	{
		const SerializedPoseEdge serial_edge = keyframe.marg_edges(i);
		edges.emplace_back();
		deserialize(serial_edge, edges.back());
	}
}


void Osmap::deserialize_edges(const SerializedKeyframeArray &serializedKeyframeArray, std::vector<OsmapKeyFrame*> &keyframes)
{
	for(size_t i=0;i<keyframes.size();i++)
	{
		const SerializedKeyframe &serializedKeyframe = serializedKeyframeArray.keyframe(i);
		KeyFrame *keyframe = keyframes[i];
		deserialize(serializedKeyframe,keyframe->mvMargEdges);

	}
}





// Kendon Varda's code to serialize many messages in one file, from https://stackoverflow.com/questions/2340730/are-there-c-equivalents-for-the-protocol-buffers-delimited-i-o-functions-in-ja
// Returns false if error, true if ok.
bool Osmap::writeDelimitedTo(
	const google::protobuf::MessageLite &message,
	google::protobuf::io::ZeroCopyOutputStream *rawOutput)
{
	// We create a new coded stream for each message.  Don't worry, this is fast.
	google::protobuf::io::CodedOutputStream output(rawOutput);

	// Write the size.
	const int size = message.ByteSize();
	output.WriteVarint32(size);
	uint8_t *buffer = output.GetDirectBufferForNBytesAndAdvance(size);
	if (buffer != NULL)
	{
		// Optimization:  The message fits in one buffer, so use the faster
		// direct-to-array serialization path.
		message.SerializeWithCachedSizesToArray(buffer);
	}
	else
	{
		// Slightly-slower path when the message is multiple buffers.
		message.SerializeWithCachedSizes(&output);
		if (output.HadError())
		{
			cerr << "Error in writeDelimitedTo." << endl;
			return false;
		}
	}
	return true;
}

bool Osmap::readDelimitedFrom(
	google::protobuf::io::ZeroCopyInputStream *rawInput,
	google::protobuf::MessageLite *message)
{
	// We create a new coded stream for each message.  Don't worry, this is fast,
	// and it makes sure the 64MB total size limit is imposed per-message rather
	// than on the whole stream.  (See the CodedInputStream interface for more
	// info on this limit.)
	google::protobuf::io::CodedInputStream input(rawInput);

	// Read the size.
	uint32_t size;
	if (!input.ReadVarint32(&size))
		return false;

	// Tell the stream not to read beyond that size.
	google::protobuf::io::CodedInputStream::Limit limit =
		input.PushLimit(size);

	// Parse the message.
	if (!message->MergeFromCodedStream(&input))
		return false;
	if (!input.ConsumedEntireMessage())
		return false;

	// Release the limit.
	input.PopLimit(limit);

	return true;
};

/*
 * Orbslam adapter.  Class wrappers.
 */
#ifndef OSMAP_DUMMY_MAP

OsmapMapPoint::OsmapMapPoint(Osmap *osmap) : MapPoint(Mat(), osmap->pRefKF, &osmap->map){};

OsmapKeyFrame::OsmapKeyFrame(Osmap *osmap) : KeyFrame(osmap->currentFrame, &osmap->map, &osmap->keyFrameDatabase){};

#else

OsmapMapPoint::OsmapMapPoint(Osmap *osmap) : MapPoint(osmap){};

OsmapKeyFrame::OsmapKeyFrame(Osmap *osmap) : KeyFrame(osmap){};

#endif

} // namespace ORB_SLAM2

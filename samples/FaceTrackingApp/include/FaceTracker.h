#pragma once

#include "Kinect.h"
#include "FaceTrackLib.h"

typedef std::shared_ptr<class FaceTracker> FaceTrackerRef;

class FaceTracker
{
public:
	static FaceTrackerRef			create();

	IFTFaceTracker*					getFaceTracker() const;
	IFTModel*						getModel() const;
	IFTResult*						getResult() const;

	bool							isTracking() const;

	void							start( const KinectSdk::DeviceOptions& deviceOptions = KinectSdk::DeviceOptions() );
	void							stop();
	void							update( const ci::Surface8u& color, const ci::Surface16u& depth, 
		const std::vector<KinectSdk::Skeleton>& skeletons );
private:
	FaceTracker();

	IFTFaceTracker*					mFaceTracker;
	IFTModel*						mModel;
	IFTResult*						mResult;
	bool							mSuccess;
	bool							mTracking;
};

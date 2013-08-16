#include "FaceTracker.h"

#include <limits>

using namespace ci;
using namespace KinectSdk;
using namespace std;

FaceTrackerRef FaceTracker::create()
{
	return FaceTrackerRef( new FaceTracker() );
}

FaceTracker::FaceTracker()
{
	mFaceTracker	= 0;
	mModel			= 0;
	mResult			= 0;
	mSuccess		= false;
	mTracking		= false; 
}

IFTFaceTracker* FaceTracker::getFaceTracker() const
{
	return mFaceTracker;
}

IFTModel* FaceTracker::getModel() const
{
	return mModel;
}

IFTResult* FaceTracker::getResult() const
{
	return mResult;
}

bool FaceTracker::isTracking() const
{
	return mTracking;
}

void FaceTracker::start( const DeviceOptions& deviceOptions )
{
	FT_CAMERA_CONFIG colorConfig;
	colorConfig.Height			= deviceOptions.getColorSize().y;
	colorConfig.Width			= deviceOptions.getColorSize().x;
	switch ( deviceOptions.getColorResolution() ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		colorConfig.FocalLength	= NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_1280x960:
		colorConfig.FocalLength	= NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.0f;
		break;
	default:
		colorConfig.FocalLength	= 0.0f;
		break;
	}

	FT_CAMERA_CONFIG depthConfig;
	depthConfig.Height			= deviceOptions.getDepthSize().y;
	depthConfig.Width			= deviceOptions.getDepthSize().x;
	switch ( deviceOptions.getDepthResolution() ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		depthConfig.FocalLength	= NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 0.25f;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		depthConfig.FocalLength	= NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		depthConfig.FocalLength	= NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.0f;
		break;
	default:
		depthConfig.FocalLength	= 0.0f;
		break;
	}

	HRESULT hr = S_OK;
	mFaceTracker = FTCreateFaceTracker();
    if ( !mFaceTracker ) {
        return; // TODO throw exception
	}

	hr = mFaceTracker->Initialize( &colorConfig, &depthConfig, 0, 0 );
	if ( FAILED( hr ) ) {
        return; // TODO throw exception
    }

	hr = mFaceTracker->CreateFTResult( &mResult );
	if ( FAILED( hr ) || mResult == 0 ) {
		return; // TODO throw exception
	}

	mTracking = true;
}

void FaceTracker::stop()
{
	mTracking = false;
}

void FaceTracker::update( const Surface8u& color, const Surface16u& depth, const vector<Skeleton>& skeletons )
{
	// TODO Put this routine in a separate thread and then fire a callback.
	//      Use a flag to keep routine fom being called while in process.
	HRESULT hr = E_FAIL;

	if ( color && depth && !skeletons.empty() ) {
		Vec3f head = Vec3f::zero();
		Vec3f neck= Vec3f::zero();
		float dist = numeric_limits<float>::max();
		for ( vector<Skeleton>::const_iterator iter = skeletons.begin(); iter != skeletons.end(); ++iter ) {
			const Skeleton& skeleton = *iter;
			if ( !skeleton.empty() ) {
				const Vec3f& h = skeleton.at( JointName::NUI_SKELETON_POSITION_HEAD ).getPosition();
				const Vec3f& n = skeleton.at( JointName::NUI_SKELETON_POSITION_HEAD ).getPosition();
				if ( h.z < dist ) {
					dist	= head.z;
					head = h;
					neck = n;
				}
			}
		}

		IFTImage* colorImage = (IFTImage*)color.getData();
		IFTImage* depthImage = (IFTImage*)depth.getChannelRed().getData();
		tagPOINT offset;
		offset.x		= 0.0f;
		offset.y		= 0.0f;
		FT_SENSOR_DATA data( colorImage, depthImage, 1.0f, offset );

		FT_VECTOR3D h( head.x, head.y, head.z );
		FT_VECTOR3D n( neck.x, neck.y, neck.z );
		FT_VECTOR3D hint[ 2 ] = { n, h };

		if ( mSuccess ) {
			hr = mFaceTracker->ContinueTracking( &data, hint, mResult );
		} else {
			hr = mFaceTracker->StartTracking( &data, 0, hint, mResult );
		}
	}

	mSuccess = SUCCEEDED( hr ) && SUCCEEDED( mResult->GetStatus() );
	if ( mSuccess ) {
		hr = mFaceTracker->GetFaceModel( &mModel );
	} else {
		mResult->Reset();
	}
}

#pragma once

#include "cinder/Exception.h"
#include "cinder/Matrix44.h"
#include "cinder/Rect.h"
#include "cinder/TriMesh.h"
#include "Kinect.h"
#include "FaceTrackLib.h"
#include <memory>
#include <thread>

typedef std::shared_ptr<class FaceTracker> FaceTrackerRef;

class FaceTracker
{
public:

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Face
	{
	public:
		Face();

		const ci::Rectf&			getBounds() const;
		const ci::TriMesh2d&		getMesh() const;
		const ci::Matrix44f&		getTransformMatrix() const;
		size_t						getUserId() const;
	protected:
		Face( size_t userId, const ci::Rectf& bounds, const ci::TriMesh2d& mesh, const ci::Matrix44f& matrix );

		ci::Rectf					mBounds;
		ci::Matrix44f				mMatrix;
		ci::TriMesh2d				mMesh;
		size_t						mUserId;

		friend class				FaceTracker;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

protected:
	typedef std::function<void( Face )>		EventHandler;
	typedef std::shared_ptr<std::thread>	ThreadRef;
public:
	static FaceTrackerRef			create();
	~FaceTracker();

	IFTFaceTracker*					getFaceTracker() const;
	IFTModel*						getModel() const;
	IFTResult*						getResult() const;

	bool							isTracking() const;

	void							start( const KinectSdk::DeviceOptions& deviceOptions = KinectSdk::DeviceOptions() );
	void							stop();
	
	void							findFace( const ci::Surface8u& color, const ci::Surface16u& depth, 
		const KinectSdk::Skeleton& skeleton = KinectSdk::Skeleton(), size_t userId = 0 );
	void							update();

	template<typename T, typename Y> 
	inline void						connectEventHander( T eventHandler, Y* obj )
	{
		connectEventHander( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	void							connectEventHander( const EventHandler& eventHandler );
protected:
	FaceTracker();

	EventHandler					mEventHandler;
	volatile bool					mNewFace;
	volatile bool					mRunning;
	ThreadRef						mThread;
	void							run();

	FT_CAMERA_CONFIG				mConfigColor;
	FT_CAMERA_CONFIG				mConfigDepth;

	Face							mFace;
	KinectSdk::Skeleton				mSkeleton;
	ci::Surface8u					mSurfaceColor;
	ci::Surface16u					mSurfaceDepth;
	size_t							mUserId;

	IFTFaceTracker*					mFaceTracker;
	IFTModel*						mModel;
	IFTResult*						mResult;
	bool							mSuccess;
public:

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Exception : public cinder::Exception
	{
	public:
		const char* what() const throw();
	protected:
		char			mMessage[ 2048 ];
		friend class	FaceTracker;
	};

	class ExcFaceTrackerCreate : public Exception 
	{
	  public:
		ExcFaceTrackerCreate() throw();
	};
	
	class ExcFaceTrackerCreateResult : public Exception 
	{
	public:
		  ExcFaceTrackerCreateResult( long hr ) throw();
	};

	class ExcFaceTrackerInit : public Exception 
	{
	public:
		ExcFaceTrackerInit( long hr ) throw();
	};
};

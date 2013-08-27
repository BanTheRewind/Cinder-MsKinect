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
	enum : size_t
	{
		AU0_UPPER_LIP_RAISER, AU1_JAW_LOWERER, AU2_LIP_STRETCHER, 
		AU3_BROW_LOWERER, AU4_LIP_CORNER_DEPRESSOR, AU5_OUTER_BROW_RAISER	
	} typedef AnimationUnit;
	typedef std::map<AnimationUnit, float>	AnimationUnitMap;

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Face
	{
	public:
		Face();

		const AnimationUnitMap&			getAnimationUnits() const;
		const ci::Rectf&				getBounds() const;
		const ci::TriMesh&				getMesh() const;
		const ci::TriMesh2d&			getMesh2d() const;
		const ci::Matrix44f&			getTransformMatrix() const;
		size_t							getUserId() const;
	protected:
		Face( size_t userId, const ci::Rectf& bounds, const ci::TriMesh& mesh, const ci::Matrix44f& matrix );

		AnimationUnitMap				mAnimationUnits;
		ci::Rectf						mBounds;
		ci::Matrix44f					mMatrix;
		ci::TriMesh						mMesh;
		ci::TriMesh2d					mMesh2d;
		size_t							mUserId;

		friend class					FaceTracker;
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

	void							enableCalcMesh( bool enabled = true );
	void							enableCalcMesh2d( bool enabled = true );
	bool							isCalcMeshEnabled() const;
	bool							isCalcMesh2dEnabled() const;

	bool							isTracking() const;

	virtual void					start( const KinectSdk::DeviceOptions& deviceOptions = KinectSdk::DeviceOptions() );
	virtual void					stop();
	
	virtual void					findFace( const ci::Surface8u& color, const ci::Surface16u& depth, 
		const ci::Vec3f headPoints[ 2 ] = 0, size_t userId = 0 );
	virtual void					update();

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
	virtual void					run();

	bool							mCalcMesh;
	bool							mCalcMesh2d;
	FT_CAMERA_CONFIG				mConfigColor;
	FT_CAMERA_CONFIG				mConfigDepth;
	Face							mFace;
	IFTFaceTracker*					mFaceTracker;
	std::vector<ci::Vec3f>			mHeadPoints;
	IFTImage*						mImageColor;
	IFTImage*						mImageDepth;
	IFTModel*						mModel;
	IFTResult*						mResult;
	bool							mSuccess;
	size_t							mUserId;
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

	class ExcFaceTrackerCreateImage : public Exception 
	{
	public:
		ExcFaceTrackerCreateImage( long hr ) throw();
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
 
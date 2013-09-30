/*
* 
* Copyright (c) 2013, Ban the Rewind, Wieden+Kennedy
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#pragma once

#include "cinder/Exception.h"
#include "cinder/Matrix44.h"
#include "cinder/Rect.h"
#include "cinder/TriMesh.h"
#include "Kinect.h"
#include "FaceTrackLib.h"
#include <memory>
#include <thread>

namespace MsKinect 
{
typedef std::shared_ptr<class FaceTracker> FaceTrackerRef;

//! Microsoft FaceTracking API wrapper for use with the Kinect.
class FaceTracker
{
public:
	/*! Animation units representing a subset of Candide3 model's 
		action units.
		http://msdn.microsoft.com/en-us/library/jj130970.aspx 
		http://www.icg.isy.liu.se/candide/ */
	enum : size_t
	{
		AU0_UPPER_LIP_RAISER, AU1_JAW_LOWERER, AU2_LIP_STRETCHER, 
		AU3_BROW_LOWERER, AU4_LIP_CORNER_DEPRESSOR, AU5_OUTER_BROW_RAISER	
	} typedef AnimationUnit;
	//! Tyoe definition for animation unit map.
	typedef std::map<AnimationUnit, float>	AnimationUnitMap;

	//////////////////////////////////////////////////////////////////////////////////////////////

	//! Structure containing face data.
	class Face
	{
	public:
		Face();

		/*! Returns animation unit (AU) map. First value is an 
			AnimationUnit enumerator. The second value is a float 
			between -1.0 and 1.0. See:
			http://msdn.microsoft.com/en-us/library/jj130970.aspx */
		const AnimationUnitMap&			getAnimationUnits() const;
		//! Returns rectangle of face location in pixels inside the color image.
		const ci::Rectf&				getBounds() const;
		/*! Returns 3D TriMesh of face in world space. FaceTracker must 
			have mesh calculation enabled. */
		const ci::TriMesh&				getMesh() const;
		/*! Returns 2D TriMesh of face. Coordinates are projected into color image.
			FaceTracker must have 2D mesh calculation enabled. */
		const ci::TriMesh2d&			getMesh2d() const;
		//! Returns transform matrix of face's pose.
		const ci::Matrix44f&			getPoseMatrix() const;
		//! Returns ID provided in FaceTracker::findFaces().
		size_t							getUserId() const;
	protected:
		AnimationUnitMap				mAnimationUnits;
		ci::Rectf						mBounds;
		ci::TriMesh						mMesh;
		ci::TriMesh2d					mMesh2d;
		ci::Matrix44f					mPoseMatrix;
		size_t							mUserId;

		friend class					FaceTracker;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

protected:
	typedef std::function<void( Face )>		EventHandler;
	typedef std::shared_ptr<std::thread>	ThreadRef;
public:
	/*! Creates pointer to instance of FaceTracker. Tracks one face at a 
		time. For multiple faces, create multiple FaceTracker instances 
		and use hinting (pass head and neck points of skeleton) when 
		finding faces. */
	static FaceTrackerRef			create();
	~FaceTracker();

	//! Returns pointer to native IFTFaceTracker.
	IFTFaceTracker*					getFaceTracker() const;
	//! Returns pointer to native IFTModel.
	IFTModel*						getModel() const;
	//! Returns pointer to native IFTResult.
	IFTResult*						getResult() const;

	/*! Enables 3D mesh calculation. Face will be returned with 
		empty TriMesh if disabled. */
	void							enableCalcMesh( bool enabled = true );
	/*! Enables 2D mesh calculation. Face will be returned with 
		empty TriMesh2d if disabled. TriMesh2d coordinates are 
		projected into color image. */
	void							enableCalcMesh2d( bool enabled = true );
	//! Returns true if 3D mesh calculation is enabled.
	bool							isCalcMeshEnabled() const;
	//! Returns true if 2D mesh calculation is enabled.
	bool							isCalcMesh2dEnabled() const;

	//! Returns true if face tracker is running.
	bool							isTracking() const;

	//! Start face tracking, allocating buffers based on \a deviceOptions.
	virtual void					start( const DeviceOptions& deviceOptions = DeviceOptions() );
	//! Stop face tracking
	virtual void					stop();
	
	/*! Update \a color and \a depth images from Kinect. Pass head and 
		neck points together, in order, through \a headPoints to target a user. 
		The value passed to \a userId will be returned from Face::getUserId() in 
		the event handler's face argument. */
	virtual void					update( const ci::Surface8u& color, const ci::Channel16u& depth, 
		const ci::Vec3f headPoints[ 2 ] = 0, size_t userId = 0 );
	
	//! Set event handler to method with signature void( FaceTracker::Face ).
	template<typename T, typename Y> 
	inline void						connectEventHander( T eventHandler, Y* obj )
	{
		connectEventHander( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	//! Set event handler to method with signature void( FaceTracker::Face ).
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
	ci::Channel16u					mChannelDepth;
	FT_CAMERA_CONFIG				mConfigColor;
	FT_CAMERA_CONFIG				mConfigDepth;
	Face							mFace;
	IFTFaceTracker*					mFaceTracker;
	std::vector<ci::Vec3f>			mHeadPoints;
	IFTModel*						mModel;
	IFTResult*						mResult;
	FT_SENSOR_DATA					mSensorData;
	bool							mSuccess;
	ci::Surface8u					mSurfaceColor;
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

	//! Exception representing failure to create FaceTracker.
	class ExcFaceTrackerCreate : public Exception 
	{
	public:
		ExcFaceTrackerCreate() throw();
	};

	//! Exception representing failure to create image buffer.
	class ExcFaceTrackerCreateImage : public Exception 
	{
	public:
		ExcFaceTrackerCreateImage( long hr ) throw();
	};
	
	//! Exception representing failure to create FaceTracker result.
	class ExcFaceTrackerCreateResult : public Exception 
	{
	public:
		  ExcFaceTrackerCreateResult( long hr ) throw();
	};

	//! Exception representing failure to initialize FaceTracker.
	class ExcFaceTrackerInit : public Exception 
	{
	public:
		ExcFaceTrackerInit( long hr ) throw();
	};
};
}
 
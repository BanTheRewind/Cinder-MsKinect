/*
* 
* Copyright (c) 2012, Ban the Rewind
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

// Includes
#include "boost/signals2.hpp"
#include "cinder/Cinder.h"
#include "cinder/Matrix.h"
#include "cinder/Quaternion.h"
#include "cinder/Surface.h"
#include "cinder/Thread.h"
#include <map>
#include "ole2.h"
#include "NuiApi.h"
#include <vector>

// Kinect SDK wrapper for Cinder
namespace KinectSdk
{

	class Kinect;
	typedef NUI_SKELETON_BONE_ROTATION		BoneRotation;
	typedef NUI_IMAGE_RESOLUTION			ImageResolution;
	typedef NUI_SKELETON_POSITION_INDEX		JointName;
	typedef std::shared_ptr<Kinect>			KinectRef;

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Bone
	{
	public:
		//! Returns rotation of the bone relative to camera coordinates.
		const ci::Quatf&		getAbsoluteRotation() const;
		//! Returns rotation matrix of the bone relative to camera coordinates.
		const ci::Matrix44f&	getAbsoluteRotationMatrix() const;
		//! Returns index of end joint.
		JointName				getEndJoint() const;
		//! Returns position of the bone's start joint.
		const ci::Vec3f&		getPosition() const;
		//! Returns rotation of the bone relative to the parent bone.
		const ci::Quatf&		getRotation() const;
		//! Returns rotation matrix of the bone relative to the parent bone.
		const ci::Matrix44f&	getRotationMatrix() const;
		//! Returns index of start joint.
		JointName				getStartJoint() const;
	private:
		Bone( const Vector4 &position, const _NUI_SKELETON_BONE_ORIENTATION &bone );
		ci::Matrix44f	mAbsRotMat;
		ci::Quatf		mAbsRotQuat;
		JointName		mJointStart;
		JointName		mJointEnd;
		ci::Vec3f		mPosition;
		ci::Matrix44f	mRotMat;
		ci::Quatf		mRotQuat;

		friend class	Kinect;
	};
	typedef std::map<JointName, Bone>	Skeleton;

	//////////////////////////////////////////////////////////////////////////////////////////////

	class DeviceOptions
	{
	public:
		//! Default settings
		DeviceOptions();

		//! Returns resolution of depth image.
		ImageResolution		getDepthResolution() const; 
		//! Returns size of depth image.
		const ci::Vec2i&	getDepthSize() const; 
		//! Returns unique ID for this device.
		const std::string&	getDeviceId() const;
		//! Returns 0-index for this device.
		int32_t				getDeviceIndex() const;
		//! Returns resolution of video image.
		ImageResolution		getVideoResolution() const; 
		//! Returns size of video image.
		const ci::Vec2i&	getVideoSize() const; 
		//! Returns true if depth tracking is enabled.
		bool				isDepthEnabled() const;
		//! Returns true if background remove is enabled.
		bool				isNearModeEnabled() const; 
		//! Returns true if seated mode is enabled.
		bool				isSeatedModeEnabled() const;
		//! Returns true if skeleton tracking is enabled.
		bool				isSkeletonTrackingEnabled() const;
		//! Returns true if color video stream is enabled.
		bool				isVideoEnabled() const;
		//! Returns true if user tracking is enabled.
		bool				isUserTrackingEnabled() const;

		//! Enables depth tracking.
		DeviceOptions&		enableDepth( bool enable = true );
		//! Enables near mode (Kinect for Windows only).
		DeviceOptions&		enableNearMode( bool enable = true ); 
		/*! Enables skeleton tracking. Set \a seatedMode to true to support seated skeletons.
		    Only available on first device running at 320x240. */
		DeviceOptions&		enableSkeletonTracking( bool enable = true, bool seatedMode = false );
		//! Enables color video stream.
		DeviceOptions&		enableVideo( bool enable = true );
		/*! Enables user tracking. Only available on first device running at 320x240. */
		DeviceOptions&		enableUserTracking( bool enable = true );
		//! Sets resolution of depth image.
		DeviceOptions&		setDepthResolution( const ImageResolution &resolution = ImageResolution::NUI_IMAGE_RESOLUTION_320x240 ); 
		//! Starts device with this unique ID.
		DeviceOptions&		setDeviceId( const std::string &id = "" ); 
		//! Starts device with this 0-index.
		DeviceOptions&		setDeviceIndex( int32_t index = 0 ); 
		//! Sets resolution of video image.
		DeviceOptions&		setVideoResolution( const ImageResolution &resolution = ImageResolution::NUI_IMAGE_RESOLUTION_640x480 ); 
	private:
		bool				mEnabledDepth;
		bool				mEnabledSeatedMode;
		bool				mEnabledSkeletonTracking;
		bool				mEnabledUserTracking;
		bool				mEnabledVideo;

		ImageResolution		mDepthResolution;
		ci::Vec2i			mDepthSize;
		ImageResolution		mVideoResolution;
		ci::Vec2i			mVideoSize;

		std::string			mDeviceId;
		int32_t				mDeviceIndex;
		bool				mEnabledNearMode;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	// Kinect sensor interface
	class Kinect
	{
	public:

		/*! Skeleton smoothing enumeration. Smoother transform improve skeleton accuracy, 
			but increase latency. */
		enum : uint_fast8_t
		{
			TRANSFORM_NONE, TRANSFORM_DEFAULT, TRANSFORM_SMOOTH, TRANSFORM_VERY_SMOOTH, TRANSFORM_MAX
		} typedef Transform;

		//! Maximum number of devices supported by the Kinect SDK.
		static const int32_t			MAXIMUM_DEVICE_COUNT	= 8;
		//! Maximum device tilt angle in positive or negative degrees.
		static const int32_t			MAXIMUM_TILT_ANGLE		= 28;

		~Kinect();

		//! Adds depth image callback
		template<typename T, typename Y> 
		inline uint32_t					addDepthCallback( T callback, Y *callbackObject )
		{
			uint32_t id = mCallbacks.empty() ? 0 : mCallbacks.rbegin()->first + 1;
			mCallbacks.insert( std::make_pair( id, CallbackRef( new Callback( mSignalDepth.connect( std::bind( callback, callbackObject, std::_1, std::_2 ) ) ) ) ) );
			return id;
		}
		//! Adds skeleton tracking callback.
		template<typename T, typename Y> 
		inline uint32_t					addSkeletonTrackingCallback( T callback, Y *callbackObject )
		{
			uint32_t id = mCallbacks.empty() ? 0 : mCallbacks.rbegin()->first + 1;
			mCallbacks.insert( std::make_pair( id, CallbackRef( new Callback( mSignalSkeleton.connect( std::bind( callback, callbackObject, std::_1, std::_2 ) ) ) ) ) );
			return id;
		}
		//! Adds video tracking callback.
		template<typename T, typename Y> 
		inline uint32_t					addVideoCallback( T callback, Y *callbackObject )
		{
			uint32_t id = mCallbacks.empty() ? 0 : mCallbacks.rbegin()->first + 1;
			mCallbacks.insert( std::make_pair( id, CallbackRef( new Callback( mSignalVideo.connect( std::bind( callback, callbackObject, std::_1, std::_2 ) ) ) ) ) );
			return id;
		}
		//! Removes callback.
		void							removeCallback( uint32_t id );

		//! Creates pointer to instance of Kinect
		static KinectRef				create();		
		//! Returns number of Kinect devices.
		static int32_t					getDeviceCount();
		//! Returns use color for user ID \a id.
		static ci::Colorf				getUserColor( uint32_t id );

		//! Start capturing using settings specified in \a deviceOptions .
		void							start( const DeviceOptions &deviceOptions = DeviceOptions() );
		//! Stop capture.
		void							stop();
		//! Sends any new data to callbacks.
		void							update();

		//! Convert depth image to binary. \a invertImage to flip black and white. Default is false.
		void							enableBinaryMode( bool enable = true, bool invertImage = false );
		//! Enables user colors. Depth tracking at 320x240 or less must be enabled. Default is true.
		void							enableUserColor( bool enable = true );
		//! Enables verbose error reporting in debug console. Default is true.
		void							enableVerbose( bool enable = true );

		//! Remove background for better user tracking.
		void							removeBackground( bool remove = true );

		//! Returns depth value as 0.0 - 1.0 float for pixel at \a pos.
		float							getDepthAt( const ci::Vec2i &pos ) const;
		//! Returns frame rate of depth image processing.
		float							getDepthFrameRate() const;
		//! Returns options object for this device.
		const DeviceOptions&			getDeviceOptions() const;
		/*! Returns skeleton data. Call Kinect::checkNewSkeletons() before this to improve performance and avoid
		    threading collisions. Sets flag to false. */
		float							getSkeletonFrameRate() const;
		//! Returns current device angle in degrees between -28 and 28.
		int32_t							getTilt();
		//! Returns number of tracked users. Depth resolution must be no more than 320x240 with user tracking enabled.
		int32_t							getUserCount();
		//! Returns frame rate of color image processing.
		float							getVideoFrameRate() const;
		
		//! Returns true is actively capturing.
		bool							isCapturing() const;

		//! Flips input horizontally if \a flipped is true.
		void							setFlipped( bool flipped = true );
		//! Returns true if input is flipped.
		bool							isFlipped() const;

		//! Returns pixel location of skeleton position in depth image.
		ci::Vec2i						getSkeletonDepthPos( const ci::Vec3f &position );
		//! Returns pixel location of skeleton position in color image.
		ci::Vec2i						getSkeletonVideoPos( const ci::Vec3f &position );

		//! Sets device angle to \a degrees. Default is 0.
		void							setTilt( int32_t degrees = 0 );

		//! Return skeleton transform type.
		int_fast8_t						getTransform() const;
		//! Sets skeleton transform type.
		void							setTransform( int_fast8_t transform = TRANSFORM_DEFAULT );

	private:
		typedef boost::signals2::connection			Callback;
		typedef std::shared_ptr<Callback>			CallbackRef;
		typedef std::map<uint32_t, CallbackRef>		CallbackList;

		static const int32_t			WAIT_TIME = 250;

		Kinect();

		template <typename T> 
		struct PixelT
		{
			T r;
			T g;
			T b;
		};
		typedef PixelT<uint8_t>			Pixel;
		typedef PixelT<uint16_t>		Pixel16u;

		struct Point
		{
			long x;
			long y;
		};

		static std::vector<ci::Colorf>	sUserColors;
		static std::vector<ci::Colorf>	getUserColors();

		void							init( bool reset = false );
		
		bool							mCapture;

		boost::signals2::signal<void ( ci::Surface16u, const DeviceOptions& )>			mSignalDepth;
		boost::signals2::signal<void ( std::vector<Skeleton>, const DeviceOptions& )>	mSignalSkeleton;
		boost::signals2::signal<void ( ci::Surface8u, const DeviceOptions& )>			mSignalVideo;
		CallbackList					mCallbacks;

		DeviceOptions					mDeviceOptions;

		float							mFrameRateDepth;
		float							mFrameRateSkeletons;
		float							mFrameRateVideo;

		bool							mBinary;
		bool							mFlipped;
		bool							mGreyScale;
		bool							mInverted;

		uint_fast8_t					mTransform;

		volatile bool					mNewDepthSurface;
		volatile bool					mNewSkeletons;
		volatile bool					mNewVideoSurface;

		ci::Surface16u					mDepthSurface;
		std::vector<Skeleton>			mSkeletons;
		ci::Surface8u					mVideoSurface;

		INuiSensor						*mSensor;
		double							mTiltRequestTime;

		bool							mIsSkeletonDevice;
		Point							mPoints[ NUI_SKELETON_POSITION_COUNT ];

		void							*mDepthStreamHandle;
		void							*mVideoStreamHandle;
		bool							openDepthStream();
		bool							openVideoStream();

		bool							mRemoveBackground;

		volatile bool					mRunning;
		std::shared_ptr<boost::thread>	mThread;
		void							run();

		Pixel16u						*mRgbDepth;
		Pixel							*mRgbVideo;
		void							pixelToDepthSurface( uint16_t *buffer );
		void							pixelToVideoSurface( uint8_t *buffer );
		Pixel16u						shortToPixel( uint16_t value );

		double							mReadTimeDepth;
		double							mReadTimeSkeletons;
		double							mReadTimeVideo;

		void							deactivateUsers();
		volatile int32_t				mUserCount;
		bool							mActiveUsers[ NUI_SKELETON_COUNT ];

		void							error( long hr );
		bool							mVerbose;
		static void						trace( const std::string &message );

		friend void CALLBACK			deviceStatus( long hr, const WCHAR *instanceName, const WCHAR *deviceId, void *data );

	};

}

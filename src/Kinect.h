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
#include "cinder/Cinder.h"
#include "cinder/Surface.h"
#include "cinder/Thread.h"
#include <map>
#include "ole2.h"
#include "NuiApi.h"
#include <vector>

// Kinect NUI wrapper for Cinder
namespace KinectSdk
{

	typedef std::shared_ptr<class Kinect>	KinectRef;
	typedef NUI_IMAGE_RESOLUTION			ImageResolution;
	typedef NUI_SKELETON_POSITION_INDEX		JointName;
	typedef std::map<JointName, ci::Vec3f>	Skeleton;

	// Kinect sensor interface
	class Kinect
	{

	public:

		//! Maximum number of devices supported by the Kinect SDK.
		static const int32_t			MAXIMUM_DEVICE_COUNT	= 8;

		//! Maximum device tilt angle in positive or negative degrees.
		static const int32_t			MAXIMUM_TILT_ANGLE		= 28;

		// Creates pointer to instance of Kinect
		static KinectRef				create();

		//! Destructor
		~Kinect();

		//! Returns number of Kinect devices.
		static int32_t					getDeviceCount();

		//! Returns use color for user ID \a id.
		static ci::Colorf				getUserColor( uint32_t id );

		/*! Start capturing on device at index \a deviceIndex (see Kinect::getDeviceIndex). Default is 0. \a videoResolution sets the video 
		    resolution. Default is 640x480. \a depthResolution sets the depth resolution. Default is 320x240. \a nearMode sets near mode 
			(Kinect for Windows only). Default is false. */
		void							start( int32_t deviceIndex = 0, 
			const ImageResolution &videoResolution = ImageResolution::NUI_IMAGE_RESOLUTION_640x480, 
			const ImageResolution &depthResolution = ImageResolution::NUI_IMAGE_RESOLUTION_320x240, 
			bool nearMode = false );
		/*! Start capturing on device with unique ID \a deviceId (see Kinect::getDeviceId). \a videoResolution sets the video resolution. Default is 
		    640x480. \a depthResolution sets the depth resolution. Default is 320x240. \a nearMode sets near mode (Kinect for Windows only). 
			Default is false. */
		void							start( std::string deviceId, 
			const ImageResolution &videoResolution = ImageResolution::NUI_IMAGE_RESOLUTION_640x480, 
			const ImageResolution &depthResolution = ImageResolution::NUI_IMAGE_RESOLUTION_320x240, 
			bool nearMode = false );
		//! Stop capture.
		void							stop();

		//! Convert depth image to binary. \a invertImage to flip black and white. Default is false.
		void							enableBinaryMode( bool enable = true, bool invertImage = false );
		//! Enables depth tracking. Call before start(). Default is true.
		void							enableDepth( bool enable = true );
		//! Enables near mode (Kinect for Windows only). Call before start(). Default is false.
		void							enableNearMode( bool enable = true );
		//! Enables skeleton tracking. Call before start(). Only available on first device running 320x240. Default is true.
		void							enableSkeletons( bool enable = true );
		//! Enables user colors. Depth tracking at 320x240 or less must be enabled. Default is true.
		void							enableUserColor( bool enable = true );
		//! Enables verbose error reporting in debug console. Default is true.
		void							enableVerbose( bool enable = true );
		//! Enables color video stream. Default is true.
		void							enableVideo( bool enable = true );

		// Remove background for better user tracking
		void							removeBackground( bool remove = true );

		//! Returns true if new depth frame is available. Sets flag to false when called.
		bool							checkNewDepthFrame();
		//! Returns true if new skeleton data is available. Sets flag to false when called.
		bool							checkNewSkeletons();
		//! Returns true if new color frame is available. Sets flag to false when called.
		bool							checkNewVideoFrame();
		//! Returns current camera angle in degrees between -28 and 28.
		int32_t							getCameraAngle();
		/* Returns 16-bit depth image (12-bit color values). Call Kinect::checkNewDepthFrame() to improve performance and avoid
		   threading collisions. Consider using Kinect::getDepthAt() in lieu of reading the depth image. */
		const ci::Surface16u&			getDepth();
		//! Returns depth value as 0.0 - 1.0 float for pixel at \a pos.
		float							getDepthAt( const ci::Vec2i &pos ) const;
		//! Returns frame rate of depth image processing.
		float							getDepthFrameRate() const;
		//! Returns unique ID for this device.
		std::string						getDeviceId() const;
		//! Returns 0-index for this device.
		int32_t							getDeviceIndex() const;
		/*! Returns vector of available skeletons. Call Kinect::checkNewSkeletons() before this to improve performance and avoid
		    threading collisions. Sets flag to false. */
		const std::vector<Skeleton>&	getSkeletons();
		//! Returns frame rate of skeleton processing.
		float							getSkeletonsFrameRate() const;
		//! Return number of tracked users. Depth resolution must be no more than 320x240 with user tracking enabled.
		int32_t							getUserCount();
		/*! Return latest color image fra,e. Call Kinect::checkNewVideoFrame() before this to improve performance and avoid
		    threading collisions. Sets flag to false. */
		const ci::Surface8u&			getVideo();
		//! Returns frame rate of color image processing.
		float							getVideoFrameRate() const;
		//! Returns true is actively capturing.
		bool							isCapturing() const;

		//! Returns pixel location of skeleton position in depth image.
		ci::Vec2i						getSkeletonDepthPos( const ci::Vec3f &position );
		//! Returns pixel location of skeleton position in color image.
		ci::Vec2i						getSkeletonVideoPos( const ci::Vec3f &position );

		//! Sets camera angle to \a degrees. Default is 0.
		void							setCameraAngle( int32_t degrees = 0 );

	private:

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
		void							start( int32_t deviceIndex, std::string deviceId, const ImageResolution &videoResolution, 
			const ImageResolution &depthResolution, bool nearMode );

		bool							mCapture;

		bool							mEnabledDepth;
		bool							mEnabledSkeletons;
		bool							mEnabledVideo;

		bool							mNewDepthFrame;
		bool							mNewSkeletons;
		bool							mNewVideoFrame;

		float							mFrameRateDepth;
		float							mFrameRateSkeletons;
		float							mFrameRateVideo;

		bool							mBinary;
		bool							mGreyScale;
		bool							mInverted;

		ci::Surface16u					mDepthSurface;
		std::vector<Skeleton>			mSkeletons;
		ci::Surface8u					mVideoSurface;

		ImageResolution					mDepthResolution;
		ImageResolution					mVideoResolution;
		int32_t							mDepthHeight;
		int32_t							mDepthWidth;
		int32_t							mVideoHeight;
		int32_t							mVideoWidth;
		void							setDepthResolution( const ImageResolution &depthResolution );
		void							setVideoResolution( const ImageResolution &videoResolution );

		std::string						mDeviceId;
		int32_t							mDeviceIndex;
		bool							mEnabledNearMode;
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
		void							pixelToDepthSurface( ci::Surface16u &surface, uint16_t *buffer );
		void							pixelToVideoSurface( ci::Surface8u &surface, uint8_t *buffer );
		Pixel16u						shortToPixel( uint16_t value );

		double							mReadTimeDepth;
		double							mReadTimeSkeletons;
		double							mReadTimeVideo;

		void							deactivateUsers();
		int32_t							mUserCount;
		bool							mActiveUsers[ NUI_SKELETON_COUNT ];

		void							error( HRESULT hr );
		bool							mVerbose;
		static void						trace( const std::string &message );

		friend void CALLBACK			deviceStatus( HRESULT hrStatus, const OLECHAR* instanceName, const OLECHAR* uniqueDeviceName, void * pUserData );

	};

}

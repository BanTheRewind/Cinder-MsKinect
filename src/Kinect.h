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

	// Alias for Kinect shared pointer
	typedef std::shared_ptr<class Kinect>	KinectRef;

	// NUI aliases
	typedef NUI_IMAGE_RESOLUTION			ImageResolution;
	typedef NUI_SKELETON_POSITION_INDEX		JointName;
	typedef std::map<JointName, ci::Vec3f>	Skeleton;

	// Kinect sensor interface
	class Kinect
	{

	public:

		// Limits
		static const int32_t			MAXIMUM_DEVICE_COUNT	= 8;
		static const int32_t			MAXIMUM_TILT_ANGLE		= 28;

		// Creates pointer to instance of Kinect
		static KinectRef				create();

		// Destructor
		~Kinect();

		static int32_t					getDeviceCount();
		static ci::Colorf				getUserColor( uint32_t id );

		// Start/stop capturing
		void							start( int32_t deviceIndex = 0, 
			const ImageResolution &videoResolution = ImageResolution::NUI_IMAGE_RESOLUTION_640x480, 
			const ImageResolution &depthResolution = ImageResolution::NUI_IMAGE_RESOLUTION_320x240, 
			bool nearMode = false );
		void							stop();

		// Flags to enable each feature
		void							enableBinaryMode( bool enable = true, bool invertImage = false );
		void							enableDepth( bool enable = true );
		void							enableNearMode( bool enable = true );
		void							enableSkeletons( bool enable = true );
		void							enableUserColor( bool enable = true );
		void							enableVideo( bool enable = true );

		// Remove background for better user tracking
		void							removeBackground( bool remove = true );

		// Getters
		bool							checkNewDepthFrame() const;
		bool							checkNewSkeletons() const;
		bool							checkNewVideoFrame() const;
		int32_t							getCameraAngle();
		const ci::Surface16u&			getDepth();
		float							getDepthAt( const ci::Vec2i &pos ) const;
		float							getDepthFrameRate() const;
		const std::vector<Skeleton>&	getSkeletons();
		float							getSkeletonsFrameRate() const;
		int32_t							getUserCount();
		const ci::Surface8u&			getVideo();
		float							getVideoFrameRate() const;
		bool							isCapturing() const;

		ci::Vec2i						getSkeletonDepthPos( const ci::Vec3f &position );
		ci::Vec2i						getSkeletonVideoPos( const ci::Vec3f &position );

		// Setters
		void							setCameraAngle( int32_t degrees = 0 );
		void							setDepthResolution( const ImageResolution &depthResolution = ImageResolution::NUI_IMAGE_RESOLUTION_320x240 );
		void							setVideoResolution( const ImageResolution &videoResolution = ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
		void							setDeviceIndex( int32_t deviceIndex = 0 );

	private:

		// Maximum wait time in milliseconds for new Kinect data
		static const int32_t			WAIT_TIME = 250;

		// Constructor
		Kinect();

		// A pixel
		template <typename T> 
		struct PixelT
		{
			T r;
			T g;
			T b;
		};
		typedef PixelT<uint8_t>			Pixel;
		typedef PixelT<uint16_t>		Pixel16u;

		// A point
		struct Point
		{
			long x;
			long y;
		};

		// User colors
		static std::vector<ci::Colorf>	sUserColors;
		static std::vector<ci::Colorf>	getUserColors();

		// Initialize properties
		void							init();
		void							restart();

		// Capturing flag
		bool							mCapture;

		bool							mEnabledDepth;
		bool							mEnabledSkeletons;
		bool							mEnabledVideo;

		// Flags if data is new
		bool							mNewDepthFrame;
		bool							mNewSkeletons;
		bool							mNewVideoFrame;

		// Frame rates
		float							mFrameRateDepth;
		float							mFrameRateSkeletons;
		float							mFrameRateVideo;

		// Binary mode
		bool							mBinary;
		bool							mGreyScale;
		bool							mInverted;

		// Kinect output data
		ci::Surface16u					mDepthSurface;
		std::vector<Skeleton>			mSkeletons;
		ci::Surface8u					mVideoSurface;

		// Image resolution
		ImageResolution					mDepthResolution;
		ImageResolution					mVideoResolution;
		int32_t							mDepthHeight;
		int32_t							mDepthWidth;
		int32_t							mVideoHeight;
		int32_t							mVideoWidth;

		// Sensor
		int32_t							mDeviceIndex;
		bool							mEnabledNearMode;
		INuiSensor						*mSensor;
		double							mTiltRequestTime;

		// Skeleton
		bool							mIsSkeletonDevice;
		Point							mPoints[ NUI_SKELETON_POSITION_COUNT ];

		// Image streams
		void							*mDepthStreamHandle;
		void							*mVideoStreamHandle;
		bool							openDepthStream();
		bool							openVideoStream();

		// Set to true to set background to black in depth image
		bool							mRemoveBackground;

		// Threading
		volatile bool					mRunning;
		std::shared_ptr<boost::thread>	mThread;
		void							run();

		// Image data
		Pixel16u						*mRgbDepth;
		Pixel							*mRgbVideo;
		void							pixelToDepthSurface( ci::Surface16u &surface, uint16_t *buffer );
		void							pixelToVideoSurface( ci::Surface8u &surface, uint8_t *buffer );
		Pixel16u						shortToPixel( uint16_t value );

		// Frame rate
		double							mReadTimeDepth;
		double							mReadTimeSkeletons;
		double							mReadTimeVideo;

		// User status
		void							deactivateUsers();
		int32_t							mUserCount;
		bool							mActiveUsers[ NUI_SKELETON_COUNT ];

		// Debug
		static void						trace( const std::string &message );

		// Allow callback to access private members
		friend void CALLBACK			deviceStatus( HRESULT hrStatus, const OLECHAR* instanceName, const OLECHAR* uniqueDeviceName, void * pUserData );

	};

}

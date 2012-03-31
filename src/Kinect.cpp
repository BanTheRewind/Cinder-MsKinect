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

// Include header
#include "Kinect.h"

#include "boost/date_time.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "cinder/app/App.h"
#include "cinder/Utilities.h"

// Kinect SDK namespace
namespace KinectSdk
{

	// Imports
	using namespace ci;
	using namespace ci::app;
	using namespace std;

	/****************************/

	// Static property implementations
	const double Kinect::TILT_REQUEST_INTERVAL = 1.5;
	vector<Colorf> Kinect::USER_COLORS = getUserColors();

	// Get color for user ID
	ci::Colorf Kinect::getUserColor( uint32_t id ) 
	{ 
		return USER_COLORS[ ci::math<uint32_t>::clamp( id, 0, 5 ) ]; 
	}

	// Defines static user color list
	vector<Colorf> Kinect::getUserColors()
	{

		// Already defined
		if ( USER_COLORS.size() == NUI_SKELETON_COUNT ) {
			return USER_COLORS;
		}

		// Define and return user colors
		vector<Colorf> colors;
		colors.push_back( Colorf( 1.0f, 0.0f, 0.0f ) );
		colors.push_back( Colorf( 0.0f, 1.0f, 0.0f ) );
		colors.push_back( Colorf( 0.25f, 1.0f, 1.0 ) );
		colors.push_back( Colorf( 1.0f, 1.0f, 0.25f ) );
		colors.push_back( Colorf( 1.0f, 0.25f, 1.0f ) );
		colors.push_back( Colorf( 0.5f, 0.5f, 1.0f ) );
		return colors;

	}

	// Creates instance of Kinect pointer
	KinectRef Kinect::create()
	{
		return KinectRef( new Kinect( ) );
	}

	/****************************/

	// Constructor
	Kinect::Kinect()
	{

		// Initialize all values
		init();

		// Initialize skeletons
		for ( int32_t i = 0; i < NUI_SKELETON_COUNT; i++ ) {
			mSkeletons.push_back( Skeleton() );
		}

	}

	// Destructor
	Kinect::~Kinect()
	{

		// Stop
		stop();

		// Clean up
		if ( mRgbDepth != 0 ) {
			delete [] mRgbDepth;
			mRgbDepth = 0;
		}
		if ( mRgbVideo != 0 ) {
			delete [] mRgbVideo;
			mRgbVideo = 0;
		}

	}

	bool Kinect::checkNewDepthFrame() const
	{ 
		return mNewDepthFrame; 
	}
	bool Kinect::checkNewSkeletons() const
	{ 
		return mNewSkeletons; 
	}
	bool Kinect::checkNewVideoFrame() const
	{ 
		return mNewVideoFrame; 
	}

	// Deactivate users
	void Kinect::deactivateUsers()
	{
		for ( uint32_t i = 0; i < NUI_SKELETON_COUNT; i++ ) {
			mActiveUsers[ i ] = false;
		}
	}

	// Set binary tracking mode
	void Kinect::enableBinaryMode( bool enable, bool invertImage )
	{
		mBinary = enable;
		mInverted = invertImage;
	}

	// Enable or disable depth tracking
	void Kinect::enableDepth( bool enable )
	{
		
		// Set user count to 0 if disabled
		if ( !enable ) {
			deactivateUsers();
		}

		// Set flag
		bool toggle = mEnabledDepth != enable;
		mEnabledDepth = enable;

		// Value has changed
		if ( toggle )
		{

			// Reset frame rate and surface
			if ( !mEnabledDepth ) {
				mFrameRateDepth = 0.0f;
			}

			// Initialize surface
			mDepthSurface = Surface8u( mDepthWidth, mDepthHeight, false, SurfaceChannelOrder::RGBA );

		}

	}

	// Enable or disable near mode
	void Kinect::enableNearMode( bool enable ) 
	{
		bool toggle = mEnabledNearMode != enable;
		mEnabledNearMode = enable;
		if ( toggle && mEnabledDepth ) {
			mEnabledDepth = false;
			enableDepth( true );
		}
	}

	// Enable or disable skeleton tracking
	void Kinect::enableSkeletons( bool enable )
	{

		// Only supports skeletons on first device
		if ( !mIsSkeletonDevice ) {
			enable = false;
		}

		// Set flag
		bool toggle = mEnabledSkeletons != enable;
		mEnabledSkeletons = enable;

		// Value has changed
		if ( toggle )
		{

			// Reset frame rate
			if ( !mEnabledSkeletons ) {
				mFrameRateSkeletons = 0.0f;
			}

			// Initialize skeletons
			mSkeletons.clear();
			for ( int32_t i = 0; i < NUI_SKELETON_COUNT; i++ ) {
				mSkeletons.push_back( Skeleton() );
			}

		}

	}

	// Enable or disable user color
	void Kinect::enableUserColor( bool enable )
	{
		mGreyScale = !enable;
	}

	// Enable or disable video tracking
	void Kinect::enableVideo( bool enable )
	{

		// Set flag
		bool toggle = mEnabledVideo != enable;
		mEnabledVideo = enable;

		// Value has changed
		if ( toggle )
		{

			// Reset frame rate
			if ( !mEnabledVideo ) {
				mFrameRateVideo = 0.0f;
			}

			// Initialize surface
			mVideoSurface = Surface8u( mVideoWidth, mVideoHeight, false, SurfaceChannelOrder::RGBA );

		}

	}

	// Get camera angle
	int32_t Kinect::getCameraAngle()
	{
		long degrees = 0L;
		if ( mCapture && mSensor != 0 ) {
			mSensor->NuiCameraElevationGetAngle( & degrees );
		}
		return (int32_t)degrees;
	}

	// Get depth surface
	const Surface8u& Kinect::getDepth()
	{ 
		mNewDepthFrame = false;
		return mDepthSurface;
	}

	float Kinect::getDepthFrameRate() const 
	{
		return mFrameRateDepth;
	}

	// Get device count
	int32_t Kinect::getDeviceCount()
	{
		int32_t deviceCount = 0;
		NuiGetSensorCount( &deviceCount );
		return deviceCount;
	}

	// Get skeletons
	const vector<Skeleton>& Kinect::getSkeletons()
	{
		mNewSkeletons = false;
		return mSkeletons;
	}

	float Kinect::getSkeletonsFrameRate() const 
	{ 
		return mFrameRateSkeletons; 
	}

	// Get user count
	int32_t Kinect::getUserCount()
	{
		return mEnabledDepth ? mUserCount : 0;
	}

	// Get video
	const Surface8u& Kinect::getVideo()
	{
		mNewVideoFrame = false;
		return mVideoSurface;
	}

	float Kinect::getVideoFrameRate() const
	{ 
		return mFrameRateVideo; 
	}

	// Initialize properties
	void Kinect::init()
	{

		// Define all properties with default values
		mBinary = false;
		mCapture = false;
		mDepthHeight = 240;
		mDepthResolution = ImageResolution::NUI_IMAGE_RESOLUTION_INVALID;
		mDepthStreamHandle = 0;
		mDepthWidth = 320;
		mDeviceIndex = 0;
		mEnabledDepth = true;
		mEnabledSkeletons = true;
		mEnabledVideo = true;
		mFrameRateDepth = 0.0f;
		mFrameRateSkeletons = 0.0f;
		mFrameRateVideo = 0.0f;
		mGreyScale = false;
		mInverted = false;
		mSensor = 0;
		mIsSkeletonDevice = false;
		mEnabledNearMode = false;
		mNewDepthFrame = false;
		mNewSkeletons = false;
		mNewVideoFrame = false;
		mReadTimeDepth = 0.0;
		mReadTimeSkeletons = 0.0;
		mReadTimeVideo = 0.0;
		mRemoveBackground = false;
		mRgbDepth = 0;
		mRgbVideo = 0;
		mTiltRequestTime = 0.0;
		mUserCount = 0;
		mVideoHeight = 480;
		mVideoResolution = ImageResolution::NUI_IMAGE_RESOLUTION_INVALID;
		mVideoStreamHandle = 0;
		mVideoWidth = 640;

		// Initialize active users
		deactivateUsers();

	}

	bool Kinect::isCapturing() const 
	{ 
		return mCapture; 
	}

	// Open depth image stream
	bool Kinect::openDepthStream()
	{
		if (mSensor != 0) {
			if ( FAILED( mSensor->NuiImageStreamOpen( mDepthResolution != ImageResolution::NUI_IMAGE_RESOLUTION_640x480 && HasSkeletalEngine( mSensor ) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH, mDepthResolution, 0, 2, 0, & mDepthStreamHandle ) ) ) {
				trace( "Unable to open depth image stream" );
				stop();
				return false;
			}
			if ( mEnabledNearMode ) {
				mSensor->NuiImageStreamSetImageFrameFlags( mDepthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE | NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES );
			}
		}
		return true;
	}

	// Open color image stream
	bool Kinect::openVideoStream()
	{
		if ( mSensor != 0 ) {
			if ( FAILED( mSensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, mVideoResolution, 0, 2, 0, &mVideoStreamHandle ) ) ) {
				trace( "Unable to open color image stream" );
				stop();
				return false;
			}
		}
		return true;
	}

	// Convert and copy pixel data to a surface
	void Kinect::pixelToSurface( Surface8u & surface, uint8_t * buffer, bool depth )
	{

		// Last frame still needs to be acquired
		if ( ( depth && mNewDepthFrame ) || ( !depth && mNewVideoFrame ) ) {
			return;
		}

		// Get dimensions
		int32_t height = surface.getHeight();
		int32_t width = surface.getWidth();
		int32_t size = width * height * 4;

		// This is depth data
		if ( depth ) {

			// Draw the bits to the bitmap
			Pixel * rgbRun = mRgbDepth;
			uint16_t * bufferRun = (uint16_t *)buffer;
			for ( int32_t y = 0; y < height; y++ ) {
				for ( int32_t x = 0 ; x < width; x++ ) {
					Pixel pixel = shortToPixel( * bufferRun );
					bufferRun++;
					* rgbRun = pixel;
					rgbRun++;
				}
			}

			// Copy depth data to surface
			memcpy( surface.getData(), (uint8_t *)mRgbDepth, size );
			mNewDepthFrame = true;

		} else {

			// Swap red/blue channels
			for ( int32_t i = 0; i < size; i += 4 ) {
				uint8_t b = buffer[ i ];
				buffer[ i ] = buffer[ i + 2 ];
				buffer[ i + 2 ] = b;
			}

			// Copy color data to surface
			memcpy( surface.getData(), buffer, size );
			mNewVideoFrame = true;

		}

	}

	// Remove background for cleaner user tracking
	void Kinect::removeBackground( bool remove )
	{
		mRemoveBackground = remove;
	}

	// Restart the device
	void Kinect::restart()
	{
		if ( mCapture ) {
			stop();
		}
		if ( !mCapture ) {
			start( mDeviceIndex, mVideoResolution, mDepthResolution, mEnabledNearMode );
		}
	}

	// Thread loop
	void Kinect::run()
	{

		// Capturing
		while ( mCapture ) {

			// Check sensor
			if ( mSensor != 0 ) {

				// Get elapsed time to calculate frame rate
				double time = getElapsedSeconds();

				//////////////////////////////////////////////////////////////////////////////////////////////

				if ( mEnabledDepth && mDepthStreamHandle != 0 && !mNewDepthFrame ) {

					// Acquire depth image
					_NUI_IMAGE_FRAME imageFrame;
					if ( FAILED( mSensor->NuiImageStreamGetNextFrame( mDepthStreamHandle, WAIT_TIME, &imageFrame ) ) ) {

						// Try opening stream if process failed -- exit if error
						if ( !openDepthStream() ) {
							mEnabledDepth = false;
						}

					} else {

						// Read texture to surface
						INuiFrameTexture * texture = imageFrame.pFrameTexture;
						_NUI_LOCKED_RECT lockedRect;
						texture->LockRect( 0, &lockedRect, 0, 0 );
						if ( lockedRect.Pitch == 0 ) {
							trace( "Invalid buffer length received" );
						} else {
							pixelToSurface( mDepthSurface, (uint8_t *)lockedRect.pBits, true );
						}

						// Clean up
						mSensor->NuiImageStreamReleaseFrame( mDepthStreamHandle, &imageFrame );

						// Update frame rate
						mFrameRateDepth = (float)( 1.0 / ( time - mReadTimeDepth ) );
						mReadTimeDepth = time;

						// Set flag
						mNewDepthFrame = true;

						// Update user count
						mUserCount = 0;
						for ( uint32_t i = 0; i < NUI_SKELETON_COUNT; i++ ) {
							if ( mActiveUsers[ i ] ) {
								mUserCount++;
							}
						}

					}

				}

				//////////////////////////////////////////////////////////////////////////////////////////////

				if ( mEnabledSkeletons && mIsSkeletonDevice && !mNewSkeletons ) {

					// Acquire skeleton
					_NUI_SKELETON_FRAME skeletonFrame;
					if ( !FAILED( mSensor->NuiSkeletonGetNextFrame( WAIT_TIME, &skeletonFrame ) ) ) {

						// Iterate through skeletons
						bool foundSkeleton = false;
						for ( int32_t i = 0; i < NUI_SKELETON_COUNT; i++ ) {

							// Clear skeleton data
							mSkeletons[ i ].clear();

							// Mark skeleton found
							if ( ( skeletonFrame.SkeletonData + i )->eTrackingState == NUI_SKELETON_TRACKED ) {

								// Smooth out the skeleton data when found
								if ( !foundSkeleton ) {
									mSensor->NuiTransformSmooth( &skeletonFrame, 0 );
									foundSkeleton = true;
								}

								// Get skeleton data
								_NUI_SKELETON_DATA skeletonData = *( skeletonFrame.SkeletonData + i );

								// Set joint data
								for ( int32_t j = 0; j < (int32_t)NUI_SKELETON_POSITION_COUNT; j++ ) {
									Vector4 point = *( skeletonData.SkeletonPositions + j );
									( mSkeletons.begin() + i )->insert( std::make_pair<JointName, Vec3f>( (JointName)j, Vec3f( point.x, point.y, point.z ) ) );
								}

							}

						}

						// Update frame rate
						mFrameRateSkeletons = (float)( 1.0 / ( time - mReadTimeSkeletons ) );
						mReadTimeSkeletons = time;

						// Set flag
						mNewSkeletons = true;

					}

				}

				//////////////////////////////////////////////////////////////////////////////////////////////

				if ( mEnabledVideo && mVideoStreamHandle != 0 && !mNewVideoFrame ) {

					// Acquire video image
					_NUI_IMAGE_FRAME imageFrame;
					if ( FAILED( mSensor->NuiImageStreamGetNextFrame( mVideoStreamHandle, WAIT_TIME, & imageFrame ) ) ) {

						// Try opening stream if process failed -- exit if error
						if ( openVideoStream() ) {
							mEnabledVideo = false;
						}

					} else {

						// Read texture
						INuiFrameTexture * texture = imageFrame.pFrameTexture;
						_NUI_LOCKED_RECT lockedRect;
						texture->LockRect( 0, & lockedRect, 0, 0 );
						if ( lockedRect.Pitch != 0 ) {
							pixelToSurface( mVideoSurface, (uint8_t *)lockedRect.pBits );
						} else {
							trace("Invalid buffer length received");
						}

						// Clean up
						mSensor->NuiImageStreamReleaseFrame( mVideoStreamHandle, &imageFrame );

						// Update frame rate
						mFrameRateVideo = (float)( 1.0 / ( time - mReadTimeVideo ) );
						mReadTimeVideo = time;

						// Set flag
						mNewVideoFrame = true;

					}

				}

			}

			// Pause thread
			Sleep( 17 );

		}

		// Return to join thread
		return;

	}

	// Set camera angle
	void Kinect::setCameraAngle( int32_t degrees )
	{

		// Tilt requests should be space apart to prevent wear on the motor
		double elapsedSeconds = getElapsedSeconds();
		if ( mCapture && mSensor != 0 && elapsedSeconds - mTiltRequestTime > TILT_REQUEST_INTERVAL ) {
			mSensor->NuiCameraElevationSetAngle( (long)math<int32_t>::clamp( degrees, -MAXIMUM_TILT_ANGLE, MAXIMUM_TILT_ANGLE ) );
			mTiltRequestTime = elapsedSeconds;
		}

	}

	// Set resolution for depth sensor
	void Kinect::setDepthResolution( const ImageResolution & depthResolution )
	{

		// Set resolution
		mDepthResolution = depthResolution;
		switch ( mDepthResolution ) {
		case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
			mDepthWidth = 640;
			mDepthHeight = 480;
			break;
		case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
			mDepthWidth = 320;
			mDepthHeight = 240;
			break;
		case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
			mDepthWidth = 80;
			mDepthHeight = 60;
			break;
		default:
			mDepthResolution = NUI_IMAGE_RESOLUTION_320x240;
			mDepthWidth = 320;
			mDepthHeight = 240;
			trace( "Invalid depth resolution specified" );
			break;
		}

		// Allocate bitmap data
		mRgbDepth = new Pixel[ mDepthWidth * mDepthHeight * 4 ];

		// Restart if capturing
		if ( mCapture ) {
			restart();
		}

	}

	// Change device index
	void Kinect::setDeviceIndex( int32_t deviceIndex )
	{

		// Bail if no change
		if ( mDeviceIndex == deviceIndex ) {
			return;
		}

		// Set device index
		mDeviceIndex = math<int32_t>::clamp( deviceIndex, 0, math<int32_t>::max( getDeviceCount() - 1, 0 ) );
		trace( "Device count: " + toString( getDeviceCount() ) );

		// Restart device if capturing
		if ( mCapture ) {
			restart();
		}

	}

	// Set resolution for video
	void Kinect::setVideoResolution( const ImageResolution & videoResolution )
	{

		// Set resolution
		mVideoResolution = videoResolution;
		switch ( mVideoResolution ) {
		case ImageResolution::NUI_IMAGE_RESOLUTION_1280x960:
			mVideoWidth = 1280;
			mVideoHeight = 960;
			break;
		case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
			mVideoWidth = 640;
			mVideoHeight = 480;
			break;
		case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
			mVideoWidth = 320;
			mVideoHeight = 240;
			break;
		default:
			mVideoResolution = NUI_IMAGE_RESOLUTION_640x480;
			mVideoWidth = 320;
			mVideoHeight = 240;
			trace( "Invalid video resolution specified" );
			break;
		}

		// Allocate bitmap data
		mRgbVideo = new Pixel[ mVideoWidth * mVideoHeight * 4 ];

		// Restart if capturing
		if ( mCapture ) {
			restart();
		}

	}

	// Convert value to short to pixel
	Kinect::Pixel Kinect::shortToPixel( uint16_t value )
	{

		// Extract depth and user values
		uint16_t realDepth = ( value & 0xfff8 ) >> 3;
		uint16_t user = value & 7;

		// Transform 13-bit depth information into an 8-bit intensity appropriate
		// for display (we disregard information in most significant bit)
		uint8_t intensity = 255 - (uint8_t)( 256 * realDepth / 0x0FFF );

		// Initialize pixel value
		Pixel pixel;
		pixel.b = 0;
		pixel.g = 0;
		pixel.r = 0;

		// Mark user active
		if ( user > 0 && user < 7 ) {
			mActiveUsers[ user - 1 ] = true;
		}

		// Binary mode
		if ( mBinary ) {

			// Set black and white values
			uint8_t backgroundColor = mInverted ? 255 : 0;
			uint8_t userColor = mInverted ? 0 : 255;

			// Set color
			if ( user == 0 || user == 7 ) {
				pixel.r = pixel.g = pixel.b = mRemoveBackground ? backgroundColor : userColor;
			} else {
				pixel.r = pixel.g = pixel.b = userColor;
			}

		} else if ( mGreyScale ) {

			// Set greyscale value
			if ( user == 0 || user == 7 ) {
				pixel.r = pixel.g = pixel.b = mRemoveBackground ? 0 : intensity;
			} else {
				pixel.r = pixel.g = pixel.b = intensity;
			}

		} else {

			// Colorize each user
			switch ( user ) {
			case 0:
				if ( !mRemoveBackground ) {
					pixel.r = intensity / 2;
					pixel.g = intensity / 2;
					pixel.b = intensity / 2;
				}
				break;
			case 1:
				pixel.r = intensity;
				break;
			case 2:
				pixel.g = intensity;
				break;
			case 3:
				pixel.r = intensity / 4;
				pixel.g = intensity;
				pixel.b = intensity;
				break;
			case 4:
				pixel.r = intensity;
				pixel.g = intensity;
				pixel.b = intensity / 4;
				break;
			case 5:
				pixel.r = intensity;
				pixel.g = intensity / 4;
				pixel.b = intensity;
				break;
			case 6:
				pixel.r = intensity / 2;
				pixel.g = intensity / 2;
				pixel.b = intensity;
				break;
			case 7:
				if ( !mRemoveBackground ) {
					pixel.r = 255 - (intensity / 2);
					pixel.g = 255 - (intensity / 2);
					pixel.b = 255 - (intensity / 2);
				}
			}

		}

		// Return pixel
		return pixel;

	}

	// Start capturing
	void Kinect::start( int32_t deviceIndex, const ImageResolution & videoResolution, const ImageResolution & depthResolution, bool nearMode )
	{

		// Don't start if already capturing
		if ( !mCapture ) {

			// Set device index
			setDeviceIndex( deviceIndex );

			// Set near mode
			mEnabledNearMode = nearMode;

			// Set resolution
			setDepthResolution( depthResolution );
			setVideoResolution( videoResolution );

			// Initialize device instance
			if ( mSensor == 0 ) {
				if ( FAILED( NuiCreateSensorByIndex( mDeviceIndex, &mSensor ) ) ) {
					trace( "Unable to create device instance " + toString( mDeviceIndex ) );
					return;
				}
			}

			// Initialize device
			unsigned long flags = NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON |  NUI_INITIALIZE_FLAG_USES_COLOR;
			if ( mDepthResolution == ImageResolution::NUI_IMAGE_RESOLUTION_640x480 || FAILED( mSensor->NuiInitialize( flags ) ) ) {
				flags = NUI_INITIALIZE_FLAG_USES_DEPTH |  NUI_INITIALIZE_FLAG_USES_COLOR;
				if ( FAILED(mSensor->NuiInitialize( flags ) ) ) {
					trace( "Unable to initialize device " + toString( mDeviceIndex ) );
					return;
				}
			}

			// Skeletons are only supported on the first device
			if ( HasSkeletalEngine( mSensor ) ) {

				// Enable skeleton tracking
				if ( FAILED( mSensor->NuiSkeletonTrackingEnable( 0, 0 ) ) ) {
					trace( "Unable to initialize skeleton tracking" );
					return;
				}

				// Set flag to true
				mIsSkeletonDevice = true;

			}

			// Open image streams
			if ( !openDepthStream() ) {
				return;
			}
			if ( !openVideoStream() ) {
				return;
			}

			// Enable or disable features
			bool enabledDepth = mEnabledDepth;
			bool enabledSkeletons = mEnabledSkeletons;
			bool enabledVideo = mEnabledVideo;
			mEnabledDepth = false;
			mEnabledSkeletons = false;
			mEnabledVideo = false;
			enableDepth( enabledDepth );
			enableSkeletons( enabledSkeletons );
			enableVideo( enabledVideo );

			// Start thread
			mCapture = true;
			mThread = std::shared_ptr<boost::thread>( new boost::thread( boost::bind( &Kinect::run, this ) ) );
		
		}

	}

	// Stop capturing
	void Kinect::stop()
	{

		// Only stop if capturing
		if ( mCapture ) {

			// Turn off features
			bool enabledDepth = mEnabledDepth;
			bool enabledSkeletons = mEnabledSkeletons;
			bool enabledVideo = mEnabledVideo;
			enableDepth( false );
			enableSkeletons( false );
			enableVideo( false );
			mEnabledDepth = enabledDepth;
			mEnabledSkeletons = enabledSkeletons;
			mEnabledVideo = enabledVideo;

		}

		// Stop thread
		mCapture = false;

		// Check instance
		if ( mSensor != 0 ) {

			// Shutdown sensor
			mSensor->NuiShutdown();
			if ( mSensor ) {
				mSensor->Release();
				mSensor = 0;
			}

		}

		// End thread
		if ( mThread ) {
			mThread->join();
		}

		// Reset data
		init();

	}

	// Debug trace
	void Kinect::trace( const string & message ) 
	{

		// Write to console and debug window
		console() << message << "\n";
		OutputDebugStringA( ( message + "\n" ).c_str() );

	}

}

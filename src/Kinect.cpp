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

#include "Kinect.h"

#include "cinder/app/App.h"
#include "cinder/Utilities.h"

#include <comutil.h>

namespace KinectSdk
{

	using namespace ci;
	using namespace ci::app;
	using namespace std;

	/****************************/

	void CALLBACK deviceStatus( HRESULT hrStatus, const OLECHAR* instanceName, const OLECHAR* uniqueDeviceId, void * pUserData )
	{
		Kinect* kinect = reinterpret_cast<Kinect*>( pUserData );
		if ( SUCCEEDED( hrStatus ) ) {
			kinect->start( kinect->mDeviceId, kinect->mVideoResolution, kinect->mDepthResolution, kinect->mEnabledNearMode );
		} else {
			kinect->error( hrStatus );
			reinterpret_cast<Kinect*>( pUserData )->stop();
		}
	}

	const double kTiltRequestInterval = 1.5;

	vector<Colorf> Kinect::sUserColors = getUserColors();

	ci::Colorf Kinect::getUserColor( uint32_t id ) 
	{ 
		return sUserColors.at( ci::math<uint32_t>::clamp( id, 0, 5 ) ); 
	}

	vector<Colorf> Kinect::getUserColors()
	{
		if ( sUserColors.size() == NUI_SKELETON_COUNT ) {
			return sUserColors;
		}

		vector<Colorf> colors;
		colors.push_back( Colorf( 0.0f, 1.0f, 1.0f ) );
		colors.push_back( Colorf( 0.0f, 0.0f, 1.0f ) );
		colors.push_back( Colorf( 0.0f, 1.0f, 0.0f ) );
		colors.push_back( Colorf( 0.0f, 0.5f, 1.0f ) );
		colors.push_back( Colorf( 0.0f, 1.0f, 0.5f ) );
		colors.push_back( Colorf( 0.0f, 0.5f, 0.5f ) );
		return colors;
	}

	KinectRef Kinect::create()
	{
		return KinectRef( new Kinect( ) );
	}

	/****************************/

	Kinect::Kinect()
	{
		NuiSetDeviceStatusCallback( &KinectSdk::deviceStatus, this );
		init();
		for ( int32_t i = 0; i < NUI_SKELETON_COUNT; i++ ) {
			mSkeletons.push_back( Skeleton() );
		}
	}

	Kinect::~Kinect()
	{
		stop();
		if ( mRgbDepth != 0 ) {
			delete [] mRgbDepth;
			mRgbDepth = 0;
		}
		if ( mRgbVideo != 0 ) {
			delete [] mRgbVideo;
			mRgbVideo = 0;
		}
	}

	bool Kinect::checkNewDepthFrame()
	{
		bool newDepthFrame = mNewDepthFrame;
		mNewDepthFrame = false;
		return newDepthFrame; 
	}
	bool Kinect::checkNewSkeletons()
	{ 
		bool newSkeletons = mNewSkeletons;
		mNewSkeletons = false;
		return newSkeletons; 
	}
	bool Kinect::checkNewVideoFrame()
	{ 
		bool newVideoFrame = mNewVideoFrame;
		mNewVideoFrame = false;
		return newVideoFrame; 
	}

	void Kinect::deactivateUsers()
	{
		for ( uint32_t i = 0; i < NUI_SKELETON_COUNT; i++ ) {
			mActiveUsers[ i ] = false;
		}
	}

	void Kinect::enableBinaryMode( bool enable, bool invertImage )
	{
		mBinary = enable;
		mInverted = invertImage;
	}

	void Kinect::enableDepth( bool enable )
	{
		if ( !enable ) {
			deactivateUsers();
		}

		bool toggle = mEnabledDepth != enable;
		mEnabledDepth = enable;
		if ( toggle ) {
			if ( !mEnabledDepth ) {
				mFrameRateDepth = 0.0f;
			}
			mDepthSurface = Surface16u( mDepthWidth, mDepthHeight, false, SurfaceChannelOrder::RGB );
		}
	}

	void Kinect::enableNearMode( bool enable ) 
	{
		bool toggle = mEnabledNearMode != enable;
		mEnabledNearMode = enable;
		if ( toggle && mEnabledDepth ) {
			mEnabledDepth = false;
			enableDepth( true );
		}
	}

	void Kinect::enableSkeletons( bool enable )
	{
		bool toggle = mEnabledSkeletons != enable;
		mEnabledSkeletons = enable;

		if ( toggle ) {
			if ( !mEnabledSkeletons ) {
				mFrameRateSkeletons = 0.0f;
			}

			mSkeletons.clear();
			for ( int32_t i = 0; i < NUI_SKELETON_COUNT; i++ ) {
				mSkeletons.push_back( Skeleton() );
			}
		}
	}

	void Kinect::enableUserColor( bool enable )
	{
		mGreyScale = !enable;
	}

	void Kinect::enableVerbose( bool enable )
	{
		mVerbose = enable;
	}

	void Kinect::enableVideo( bool enable )
	{
		bool toggle = mEnabledVideo != enable;
		mEnabledVideo = enable;
		if ( toggle ) {
			if ( !mEnabledVideo ) {
				mFrameRateVideo = 0.0f;
			}
			mVideoSurface = Surface8u( mVideoWidth, mVideoHeight, false, SurfaceChannelOrder::RGBA );
		}
	}

	void Kinect::error( HRESULT hr ) {
		if ( !mVerbose ) {
			return;
		}
		switch ( hr ) {
		case E_POINTER:
			trace( "Bad pointer." );
			break;
		case E_INVALIDARG:
			trace( "Invalid argument." );
			break;
		case E_NUI_DEVICE_NOT_READY:
			trace( "Device not ready." );
			break;
		case E_NUI_FEATURE_NOT_INITIALIZED:
			trace( "Feature not initialized." );
			break;
		case E_NUI_NOTCONNECTED:
			trace( "Unable to connect to device." );
			break;
		case E_FAIL:
			trace( "Attempt failed." );
			break;
		case E_NUI_IMAGE_STREAM_IN_USE:
			trace( "Image stream already in use." );
			break;
		case E_NUI_FRAME_NO_DATA:
			trace( "No frame data available" );
			break;
		case E_OUTOFMEMORY:
			trace( "Out of memory (maximum number of Kinect devices may have been reached)." );
			break;
		case ERROR_TOO_MANY_CMDS:
			trace( "Too many commands sent. Angle change requests must be made at least 1.5s apart." );
			break;
		case ERROR_RETRY:
			trace( "Device is busy.  Retry in a moment." );
			break;
		case S_OK:
			break;
		default:
			trace( "Unknown error (Code " + toString( hr ) + ")" );
		}
	}

	int32_t Kinect::getCameraAngle()
	{
		long degrees = 0L;
		if ( mCapture && mSensor != 0 ) {
			HRESULT hr = mSensor->NuiCameraElevationGetAngle( &degrees );
			if ( FAILED( hr ) ) {
				trace( "Unable to retrieve device angle:" );
				error( hr );
			}
		}
		return (int32_t)degrees;
	}

	const Surface16u& Kinect::getDepth()
	{ 
		mNewDepthFrame = false;
		return mDepthSurface;
	}

	float Kinect::getDepthAt( const ci::Vec2i &pos ) const
	{
		float depthNorm = 0.0f;
		if ( mDepthSurface ) {
			uint16_t depth = mDepthSurface.getPixel( pos ).r;
			depthNorm = 2.0f * ( (float)( depth / ( 1.0 * 0x8000 ) ) - 1.5f );
		}
		return depthNorm;
	}

	float Kinect::getDepthFrameRate() const 
	{
		return mFrameRateDepth;
	}
	
	int32_t Kinect::getDeviceCount()
	{
		int32_t deviceCount = 0;
		NuiGetSensorCount( &deviceCount );
		return deviceCount;
	}

	string Kinect::getDeviceId() const
	{
		return mDeviceId;
	}

	int32_t Kinect::getDeviceIndex() const
	{
		return mDeviceIndex;
	}

	const vector<Skeleton>& Kinect::getSkeletons()
	{
		mNewSkeletons = false;
		return mSkeletons;
	}

	float Kinect::getSkeletonsFrameRate() const 
	{ 
		return mFrameRateSkeletons; 
	}

	int32_t Kinect::getUserCount()
	{
		return mEnabledDepth ? mUserCount : 0;
	}

	const Surface8u& Kinect::getVideo()
	{
		mNewVideoFrame = false;
		return mVideoSurface;
	}

	float Kinect::getVideoFrameRate() const
	{ 
		return mFrameRateVideo; 
	}

	void Kinect::init( bool reset )
	{

		// Only set these when first initializing the device
		if ( !reset ) {
			mBinary = false;
			mDepthHeight = 240;
			mDepthResolution = ImageResolution::NUI_IMAGE_RESOLUTION_INVALID;
			mDepthWidth = 320;
			mDeviceIndex = 0;
			mDeviceId = "";
			mEnabledDepth = true;
			mEnabledNearMode = false;
			mEnabledSkeletons = true;
			mEnabledVideo = true;
			mGreyScale = false;
			mInverted = false;
			mRemoveBackground = false;
			mVerbose = true;
			mVideoHeight = 480;
			mVideoResolution = ImageResolution::NUI_IMAGE_RESOLUTION_INVALID;
			mVideoWidth = 640;
		}

		mCapture = false;
		mDepthStreamHandle = 0;
		mFrameRateDepth = 0.0f;
		mFrameRateSkeletons = 0.0f;
		mFrameRateVideo = 0.0f;
		mSensor = 0;
		mIsSkeletonDevice = false;
		mNewDepthFrame = false;
		mNewSkeletons = false;
		mNewVideoFrame = false;
		mReadTimeDepth = 0.0;
		mReadTimeSkeletons = 0.0;
		mReadTimeVideo = 0.0;
		mRgbDepth = 0;
		mRgbVideo = 0;
		mTiltRequestTime = 0.0;
		mUserCount = 0;
		mVideoStreamHandle = 0;

		deactivateUsers();

	}

	bool Kinect::isCapturing() const 
	{ 
		return mCapture; 
	}

	Vec2i Kinect::getSkeletonDepthPos( const ci::Vec3f &position )
	{
		float x;
		float y;
		Vector4 pos;
		pos.x = position.x;
		pos.y = position.y;
		pos.z = position.z;
		pos.w = 0.0f;
		NuiTransformSkeletonToDepthImage( pos, &x, &y, mDepthResolution );
		return Vec2i( (int32_t)x, (int32_t)y );
	}

	Vec2i Kinect::getSkeletonVideoPos( const ci::Vec3f &position )
	{
		float x;
		float y;
		Vector4 pos;
		pos.x = position.x;
		pos.y = position.y;
		pos.z = position.z;
		pos.w = 0.0f;
		NuiTransformSkeletonToDepthImage( pos, &x, &y, mVideoResolution );
		return Vec2i( (int32_t)x, (int32_t)y );
	}

	bool Kinect::openDepthStream()
	{
		if ( mSensor != 0) {
			HRESULT hr = mSensor->NuiImageStreamOpen( mDepthResolution != ImageResolution::NUI_IMAGE_RESOLUTION_640x480 && 
				HasSkeletalEngine( mSensor ) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH, mDepthResolution, 0, 2, 0, &mDepthStreamHandle );;
			if ( FAILED( hr ) ) {
				trace( "Unable to open depth image stream: " );
				error( hr );
				stop();
				return false;
			}
		}
		return true;
	}

	bool Kinect::openVideoStream()
	{
		if ( mSensor != 0 ) {
			HRESULT hr = mSensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, mVideoResolution, 0, 2, 0, &mVideoStreamHandle );
			if ( FAILED( hr ) ) {
				trace( "Unable to open color image stream: " );
				error( hr );
				stop();
				return false;
			}
		}
		return true;
	}

	void Kinect::pixelToDepthSurface( Surface16u &surface, uint16_t *buffer )
	{
		if ( mNewDepthFrame ) {
			return;
		}

		int32_t height = surface.getHeight();
		int32_t width = surface.getWidth();
		int32_t size = width * height * 3 * 2;

		Pixel16u* rgbRun = mRgbDepth;
		uint16_t* bufferRun = buffer;
		for ( int32_t i = 0; i < width * height; i++ ) {
			Pixel16u pixel = shortToPixel( *bufferRun );
			bufferRun++;
			*rgbRun = pixel;
			rgbRun++;
		}

		memcpy( surface.getData(), mRgbDepth, size );
		mNewDepthFrame = true;
	}

	void Kinect::pixelToVideoSurface( Surface8u &surface, uint8_t *buffer )
	{
		if ( mNewVideoFrame ) {
			return;
		}

		int32_t height = surface.getHeight();
		int32_t width = surface.getWidth();
		int32_t size = width * height * 4;

		// Swap red/blue channels
		for ( int32_t i = 0; i < size; i += 4 ) {
			uint8_t b = buffer[ i ];
			buffer[ i ] = buffer[ i + 2 ];
			buffer[ i + 2 ] = b;
		}

		memcpy( surface.getData(), buffer, size );
		mNewVideoFrame = true;
	}

	void Kinect::removeBackground( bool remove )
	{
		mRemoveBackground = remove;
	}

	void Kinect::run()
	{
		while ( mCapture ) {
			if ( mSensor != 0 ) {

				// Get elapsed time to calculate frame rate
				double time = getElapsedSeconds();

				//////////////////////////////////////////////////////////////////////////////////////////////

				if ( mEnabledDepth && mDepthStreamHandle != 0 && !mNewDepthFrame ) {

					// Acquire depth image
					_NUI_IMAGE_FRAME imageFrame;
					HRESULT hr = mSensor->NuiImageStreamGetNextFrame( mDepthStreamHandle, WAIT_TIME, &imageFrame );
					if ( FAILED( hr ) ) {
						error( hr );
					} else {

						// Read texture to surface
						INuiFrameTexture * texture = imageFrame.pFrameTexture;
						_NUI_LOCKED_RECT lockedRect;
						hr = texture->LockRect( 0, &lockedRect, 0, 0 );
						if ( FAILED( hr ) ) {
							error( hr );
						}
						if ( lockedRect.Pitch == 0 ) {
							trace( "Invalid buffer length received" );
						} else {
							pixelToDepthSurface( mDepthSurface, (uint16_t*)lockedRect.pBits );
						}

						// Clean up
						hr = mSensor->NuiImageStreamReleaseFrame( mDepthStreamHandle, &imageFrame );
						if ( FAILED( hr ) ) {
							error( hr ); 
						}
						
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
					HRESULT hr = mSensor->NuiSkeletonGetNextFrame( WAIT_TIME, &skeletonFrame );
					if ( FAILED( hr ) ) {
						error( hr );
					} else {

						// Iterate through skeletons
						bool foundSkeleton = false;
						for ( int32_t i = 0; i < NUI_SKELETON_COUNT; i++ ) {

							// Clear skeleton data
							mSkeletons[ i ].clear();

							// Mark skeleton found
							if ( ( skeletonFrame.SkeletonData + i )->eTrackingState == NUI_SKELETON_TRACKED ) {

								// Smooth out the skeleton data when found
								if ( !foundSkeleton ) {
									hr = mSensor->NuiTransformSmooth( &skeletonFrame, 0 );
									if ( FAILED( hr ) ) {
										error( hr );
									}
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
					HRESULT hr = mSensor->NuiImageStreamGetNextFrame( mVideoStreamHandle, WAIT_TIME, &imageFrame );
					if ( FAILED( hr ) ) {
						error( hr );
					} else {

						// Read texture
						INuiFrameTexture * texture = imageFrame.pFrameTexture;
						_NUI_LOCKED_RECT lockedRect;
						hr = texture->LockRect( 0, &lockedRect, 0, 0 );
						if ( FAILED( hr ) ) {
							error( hr );
						}
						if ( lockedRect.Pitch != 0 ) {
							pixelToVideoSurface( mVideoSurface, (uint8_t *)lockedRect.pBits );
						} else {
							trace( "Invalid buffer length received." );
						}

						// Clean up
						hr = mSensor->NuiImageStreamReleaseFrame( mVideoStreamHandle, &imageFrame );
						if ( FAILED( hr ) ) {
							error( hr );
						}

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

	void Kinect::setCameraAngle( int32_t degrees )
	{

		// Tilt requests should be spaced apart to prevent wear on the motor
		double elapsedSeconds = getElapsedSeconds();
		if ( mCapture && mSensor != 0 && elapsedSeconds - mTiltRequestTime > kTiltRequestInterval ) {
			HRESULT hr = mSensor->NuiCameraElevationSetAngle( (long)math<int32_t>::clamp( degrees, -MAXIMUM_TILT_ANGLE, MAXIMUM_TILT_ANGLE ) );
			if ( FAILED( hr ) ) {
				trace( "Unable to change device angle: " );
				error( hr );
			}
			mTiltRequestTime = elapsedSeconds;
		}

	}

	void Kinect::setDepthResolution( const ImageResolution &depthResolution )
	{
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
		mRgbDepth = new Pixel16u[ mDepthWidth * mDepthHeight * 4 ];
	}

	void Kinect::setVideoResolution( const ImageResolution &videoResolution )
	{
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
		mRgbVideo = new Pixel[ mVideoWidth * mVideoHeight * 4 ];
	}

	Kinect::Pixel16u Kinect::shortToPixel( uint16_t value )
	{

		// Extract depth and user values
		uint16_t depth = 0xFFFF - 0x10000 * ( ( value & 0xFFF8 ) >> 3 ) / 0x0FFF;
		uint16_t user = value & 7;

		Pixel16u pixel;
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
			uint16_t backgroundColor = mInverted ? 0xFFFF : 0;
			uint16_t userColor = mInverted ? 0 : 0xFFFF;

			// Set color
			if ( user == 0 || user == 7 ) {
				pixel.r = pixel.g = pixel.b = mRemoveBackground ? backgroundColor : userColor;
			} else {
				pixel.r = pixel.g = pixel.b = userColor;
			}

		} else if ( mGreyScale ) {

			// Set greyscale value
			if ( user == 0 || user == 7 ) {
				pixel.r = mRemoveBackground ? 0 : depth;
			} else {
				pixel.r = depth;
			}
			pixel.g = pixel.r;
			pixel.b = pixel.g;

		} else {

			// Colorize each user
			switch ( user ) {
			case 0:
				if ( !mRemoveBackground ) {
					pixel.r = depth / 4;
					pixel.g = pixel.r;
					pixel.b = pixel.g;
				}
				break;
			case 1:
				pixel.r = depth;
				break;
			case 2:
				pixel.r = depth;
				pixel.g = depth;
				break;
			case 3:
				pixel.r = depth;
				pixel.b = depth;
				break;
			case 4:
				pixel.r = depth;
				pixel.g = depth / 2;
				break;
			case 5:
				pixel.r = depth;
				pixel.b = depth / 2;
				break;
			case 6:
				pixel.r = depth;
				pixel.g = depth / 2;
				pixel.b = pixel.g;
				break;
			case 7:
				if ( !mRemoveBackground ) {
					pixel.r = 0xFFFF - ( depth / 2 );
					pixel.g = pixel.r;
					pixel.b = pixel.g;
				}
			}

		}

		// Invert image
		pixel.r = 0xFFFF - pixel.r;
		pixel.g = 0xFFFF - pixel.g;
		pixel.b = 0xFFFF - pixel.b;

		return pixel;

	}

	void Kinect::start( int32_t deviceIndex, const ImageResolution &videoResolution, const ImageResolution &depthResolution, bool nearMode )
	{
		start( deviceIndex, "", videoResolution, depthResolution, nearMode );
	}
	void Kinect::start( std::string deviceId, const ImageResolution &videoResolution, const ImageResolution &depthResolution, bool nearMode )
	{
		start( -1, deviceId, videoResolution, depthResolution, nearMode );
	}
	void Kinect::start( int32_t deviceIndex, std::string deviceId, const ImageResolution &videoResolution, 
		const ImageResolution &depthResolution, bool nearMode ) 
	{
		if ( !mCapture ) {

			// Set near mode (Kinect for Windows only)
			mEnabledNearMode = nearMode;

			// Clamp device ID and init surfaces
			if ( deviceIndex >= 0 ) {
				mDeviceIndex = math<int32_t>::clamp( deviceIndex, 0, math<int32_t>::max( getDeviceCount() - 1, 0 ) );
			}
			setDepthResolution( depthResolution );
			setVideoResolution( videoResolution );

			// Initialize device instance
			HRESULT hr;
			if ( mDeviceIndex >= 0 ) {
				hr = NuiCreateSensorByIndex( mDeviceIndex, &mSensor );
				if ( FAILED( hr ) ) {
					trace( "Unable to create device instance " + toString( mDeviceIndex ) + ": " );
					error( hr );
					return;
				}
			} else if ( deviceId.length() > 0 ) {
				_bstr_t id = deviceId.c_str();
				hr = NuiCreateSensorById( id, &mSensor );
				if ( FAILED( hr ) ) {
					trace( "Unable to create device instance " + deviceId + ":" );
					error( hr );
					return;
				}
			} else {
				trace( "Invalid device name or index." );
				return;
			}

			// Check device
			hr = mSensor != 0 ? mSensor->NuiStatus() : E_NUI_NOTCONNECTED;
			if ( hr == E_NUI_NOTCONNECTED ) {
				error( hr );
				return;
			}

			// Get device name and index
			if ( mSensor != 0 ) {
				mDeviceIndex = mSensor->NuiInstanceIndex();
				mSensor->NuiUniqueId();
				BSTR id = ::SysAllocString( mSensor->NuiUniqueId() ); 
				_bstr_t idStr( id );
				if ( idStr.length() > 0 ) {
					std::string str( idStr );
					mDeviceId = str;
				}
				::SysFreeString( id );
			} else {
				mDeviceIndex = -1;
				mDeviceId = "";
			}

			// Initialize sensor image streams
			unsigned long flags;
			if ( mDepthResolution == ImageResolution::NUI_IMAGE_RESOLUTION_640x480 ) {
				flags = NUI_INITIALIZE_FLAG_USES_DEPTH;
			} else {
				flags = NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
				if ( mEnabledSkeletons ) {
					flags |= NUI_INITIALIZE_FLAG_USES_SKELETON;
				}
			}
			if ( mEnabledVideo ) {
				flags |= NUI_INITIALIZE_FLAG_USES_COLOR;
			}
			hr = mSensor->NuiInitialize( flags );
			if ( FAILED( hr ) ) {
				trace( "Unable to initialize device " + mDeviceId + ":" );
				error( hr );
				return;
			}

			// Skeletons are only supported on the first device
			if ( mEnabledSkeletons && HasSkeletalEngine( mSensor ) ) {
				hr = mSensor->NuiSkeletonTrackingEnable( 0, 0 );
				if ( FAILED( hr ) ) {
					trace( "Unable to initialize skeleton tracking for device " + mDeviceId + ": " );
					error( hr );
					return;
				}
				mIsSkeletonDevice = true;
			}

			// Open image streams
			if ( mEnabledDepth && !openDepthStream() ) {
				return;
			}
			if ( mEnabledVideo && !openVideoStream() ) {
				return;
			}

			// Set image stream flags
			flags = NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM;
			if ( mEnabledNearMode ) {
				flags |= NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE | NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES;
			}
			hr = mSensor->NuiImageStreamSetImageFrameFlags( mDepthStreamHandle, flags );
			if ( FAILED( hr ) ) {
				trace( "Unable to set image frame flags: " );
				error( hr );
			}

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

	void Kinect::stop()
	{
		mCapture = false;
		if ( mThread ) {
			mThread->join();
			mThread.reset();
		}
		if ( mSensor != 0 ) {
			mSensor->NuiShutdown();
			if ( mSensor ) {
				mSensor->Release();
				mSensor = 0;
				mDepthStreamHandle = 0;
				mVideoStreamHandle = 0;
			}
		}
		init( true );
	}

	void Kinect::trace( const string &message ) 
	{
		console() << message << "\n";
		OutputDebugStringA( ( message + "\n" ).c_str() );
	}

}

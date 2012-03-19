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

// Includes
#include <algorithm>
#include "AudioInput.h"
#include "boost/algorithm/string.hpp"
#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "Kinect.h"

/*
* This application explores the features of the Kinect SDK wrapper. It 
* demonstrates how to start a device, query for devices, adjust tilt, 
* and read and represent audio, depth, video, and skeleton data.  
* It's also useful as a device test and control panel.
*/
class KinectApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void prepareSettings( ci::app::AppBasic::Settings * settings );
	void setup();
	void shutdown();
	void update();

	// Audio callback
	void onData( float * data, int32_t size );

private:

	// Capturing flag
	bool	mCapture;
	bool	mCapturePrev;
	bool	mBinaryMode;
	bool	mBinaryModePrev;
	bool	mEnabledAudio;
	bool	mEnabledAudioPrev;
	bool	mEnabledNearMode;
	bool	mEnabledNearModePrev;
	bool	mEnabledDepth;
	bool	mEnabledDepthPrev;
	bool	mEnabledSkeletons;
	bool	mEnabledSkeletonsPrev;
	bool	mEnabledStats;
	bool	mEnabledVideo;
	bool	mEnabledVideoPrev;
	bool	mInverted;
	bool	mInvertedPrev;

	// Audio input
	int32_t			mCallbackId;
	float *			mData;
	AudioInputRef	mInput;

	// Kinect
	void											drawSegment( const KinectSdk::Skeleton & skeleton, 
																 const std::vector<KinectSdk::JointName> & joints );
	int32_t											mCameraAngle;
	int32_t											mCameraAnglePrev;
	ci::Surface8u									mDepthSurface;
	int32_t											mDeviceCount;
	int32_t											mDeviceIndex;
	int32_t											mDeviceIndexPrev;
	std::vector<KinectSdk::KinectRef>				mDevices;
	std::vector<KinectSdk::Skeleton>				mSkeletons;
	int32_t											mUserCount;
	ci::Surface8u									mVideoSurface;
	void											startAudio();
	void											startKinect();
	void											stopAudio();

	// Skeleton segments
	void											defineBody();
	std::vector<KinectSdk::JointName>				mBody;
	std::vector<KinectSdk::JointName>				mLeftArm;
	std::vector<KinectSdk::JointName>				mLeftLeg;
	std::vector<KinectSdk::JointName>				mRightArm;
	std::vector<KinectSdk::JointName>				mRightLeg;
	std::vector<std::vector<KinectSdk::JointName> >	mSegments;

	// Camera
	ci::CameraPersp	mCamera;

	// Params
	float					mFrameRateApp;
	float					mFrameRateDepth;
	float					mFrameRateSkeletons;
	float					mFrameRateVideo;
	bool					mFullScreen;
	ci::params::InterfaceGl	mParams;
	bool					mRemoveBackground;
	bool					mRemoveBackgroundPrev;
	void					resetStats();

	// Save screen shot
	void	screenShot();

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// Define body drawing
void KinectApp::defineBody()
{

	// Bail if defined
	if ( mSegments.size() > 0 ) {
		return;
	}
	
	// Body
	mBody.push_back( NUI_SKELETON_POSITION_HIP_CENTER );
	mBody.push_back( NUI_SKELETON_POSITION_SPINE );
	mBody.push_back( NUI_SKELETON_POSITION_SHOULDER_CENTER );
	mBody.push_back( NUI_SKELETON_POSITION_HEAD );

	// Left arm
	mLeftArm.push_back( NUI_SKELETON_POSITION_SHOULDER_CENTER );
	mLeftArm.push_back( NUI_SKELETON_POSITION_SHOULDER_LEFT );
	mLeftArm.push_back( NUI_SKELETON_POSITION_ELBOW_LEFT );
	mLeftArm.push_back( NUI_SKELETON_POSITION_WRIST_LEFT );
	mLeftArm.push_back( NUI_SKELETON_POSITION_HAND_LEFT );

	// Left leg
	mLeftLeg.push_back( NUI_SKELETON_POSITION_HIP_CENTER );
	mLeftLeg.push_back( NUI_SKELETON_POSITION_HIP_LEFT );
	mLeftLeg.push_back( NUI_SKELETON_POSITION_KNEE_LEFT );
	mLeftLeg.push_back( NUI_SKELETON_POSITION_ANKLE_LEFT );
	mLeftLeg.push_back( NUI_SKELETON_POSITION_FOOT_LEFT );

	// Right arm
	mRightArm.push_back( NUI_SKELETON_POSITION_SHOULDER_CENTER );
	mRightArm.push_back( NUI_SKELETON_POSITION_SHOULDER_RIGHT );
	mRightArm.push_back( NUI_SKELETON_POSITION_ELBOW_RIGHT );
	mRightArm.push_back( NUI_SKELETON_POSITION_WRIST_RIGHT );
	mRightArm.push_back( NUI_SKELETON_POSITION_HAND_RIGHT );

	// Right leg
	mRightLeg.push_back( NUI_SKELETON_POSITION_HIP_CENTER);
	mRightLeg.push_back( NUI_SKELETON_POSITION_HIP_RIGHT );
	mRightLeg.push_back( NUI_SKELETON_POSITION_KNEE_RIGHT );
	mRightLeg.push_back( NUI_SKELETON_POSITION_ANKLE_RIGHT );
	mRightLeg.push_back( NUI_SKELETON_POSITION_FOOT_RIGHT );

	// Build skeleton drawing list
	mSegments.push_back( mBody );
	mSegments.push_back( mLeftArm );
	mSegments.push_back( mLeftLeg );
	mSegments.push_back( mRightArm );
	mSegments.push_back( mRightLeg );

}

// Render
void KinectApp::draw()
{

	// Clear window
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf( 0.1f, 0.1f, 0.1f ) );

	// We're capturing
	if ( mDevices[ mDeviceIndex ]->isCapturing() ) {

		// Set up camera for 3D
		gl::setMatrices( mCamera );

		// Move skeletons down below the rest of the interface
		gl::pushMatrices();
		gl::translate( 0.0f, -0.62f, 0.0f );

		// Iterate through skeletons
		uint32_t i = 0;
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.cbegin(); skeletonIt != mSkeletons.cend(); ++skeletonIt, i++ ) {

			// Skeleton is valid when all joints are present
			if ( skeletonIt->size() == JointName::NUI_SKELETON_POSITION_COUNT ) {

				// Set color
				gl::color( mDevices [mDeviceIndex ]->getUserColor( i ) );

				// Draw joints
				for ( Skeleton::const_iterator jointIt = skeletonIt->cbegin(); jointIt != skeletonIt->cend(); ++jointIt ) {
					gl::drawSphere( jointIt->second * Vec3f( -1.0f, 1.0f, 1.0f ), 0.025f, 16 );
				}

				// Draw body
				for ( vector<vector<JointName> >::const_iterator segmentIt = mSegments.cbegin(); segmentIt != mSegments.cend(); ++segmentIt ) {
					drawSegment( * skeletonIt, * segmentIt );
				}

			}

		}

		// Switch to 2D
		gl::popMatrices();
		gl::setMatricesWindow( getWindowSize(), true );

		// Draw depth and video textures
		gl::color( Colorf::white() );
		if ( mDepthSurface ) {
			Area srcArea( 0, 0, mDepthSurface.getWidth(), mDepthSurface.getHeight() );
			Rectf destRect( 265.0f, 15.0f, 505.0f, 195.0f );
			gl::draw( gl::Texture( mDepthSurface ), srcArea, destRect );
		}
		if ( mVideoSurface ) {
			Area srcArea( 0, 0, mVideoSurface.getWidth(), mVideoSurface.getHeight() );
			Rectf destRect( 508.0f, 15.0f, 748.0f, 195.0f );
			gl::draw( gl::Texture( mVideoSurface ), srcArea, destRect);
		}

	}

	// Check audio data
	if (mData != 0) {

		// Get dimensions
		int32_t dataSize = mInput->getDataSize();
		float scale = 240.0f / (float)dataSize;
		float height = 180.0f;
		Vec2f position( 751.0f, 15.0f );

		// Draw background
		gl::color( ColorAf::black() );
		Rectf background( position.x, position.y, position.x + 240.0f, position.y + 180.0f );
		gl::drawSolidRect( background );

		// Draw audio input
		gl::color( ColorAf::white() );
		PolyLine<Vec2f> mLine;
		for ( int32_t i = 0; i < dataSize; i++ ) {
			mLine.push_back( position + Vec2f( i * scale, math<float>::clamp( mData[ i ], -1.0f, 1.0f ) * height * 0.5f + height * 0.5f ) );
		}
		gl::draw( mLine );

	}

	// Draw the interface
	params::InterfaceGl::draw();

}

// Draw segment
void KinectApp::drawSegment( const Skeleton & skeleton, const vector<JointName> & joints )
{

	// Draw lines between each joint
	glBegin( GL_LINES );
	for ( uint32_t i = 0; i < joints.size() - 1; i++ ) {
		gl::vertex( skeleton.at( joints[ i ] ) * Vec3f( -1.0f, 1.0f, 1.0f ) );
		gl::vertex( skeleton.at( joints[ i + 1 ] ) * Vec3f( -1.0f, 1.0f, 1.0f ) );
	}
	glEnd();

}

// Called when audio buffer is full
void KinectApp::onData( float * data, int32_t size )
{

	// Get data
	mData = data;

}

// Prepare window
void KinectApp::prepareSettings( Settings * settings )
{

	// DO IT!
	settings->setWindowSize( 1005, 570 );
	settings->setFrameRate( 60.0f );

}

// Reset statistics
void KinectApp::resetStats()
{

	// Zero values
	mFrameRateDepth = 0.0f;
	mFrameRateSkeletons = 0.0f;
	mFrameRateVideo = 0.0f;
	mUserCount = 0;

}

// Take screen shot
void KinectApp::screenShot()
{

	// DO IT!
	writeImage( getAppPath() + "frame" + toString( getElapsedFrames() ) + ".png", copyWindowSurface() );

}

// Set up
void KinectApp::setup()
{

	// Set up OpenGL
	glLineWidth( 2.0f );
	gl::color( ColorAf::white() );

	// Set up camera
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, -3.0f ), Vec3f::zero() );
	mCamera.setPerspective( 45.0f, getWindowAspectRatio(), 1.0f, 1000.0f );

	// Define drawing body
	defineBody();

	// Initialize parameters
	mBinaryMode = false;
	mBinaryModePrev = mBinaryMode;
	mCapture = true;
	mCapturePrev = mCapture;
	mDeviceCount = 0;
	mDeviceIndex = 0;
	mDeviceIndexPrev = mDeviceIndex;
	mEnabledAudio = true;
	mEnabledAudioPrev = true;
	mEnabledDepth = true;
	mEnabledDepthPrev = mEnabledDepth;
	mEnabledNearMode = false;
	mEnabledNearModePrev = mEnabledNearMode;
	mEnabledSkeletons = true;
	mEnabledSkeletonsPrev = mEnabledSkeletons;
	mEnabledStats = true;
	mEnabledVideo = true;
	mEnabledVideoPrev = mEnabledVideo;
	mFrameRateApp = 0.0f;
	mFrameRateDepth = 0.0f;
	mFrameRateSkeletons = 0.0f;
	mFrameRateVideo = 0.0f;
	mFullScreen = isFullScreen();
	mInverted = false;
	mInvertedPrev = mInverted;
	mRemoveBackground = true;
	mRemoveBackgroundPrev = mRemoveBackground;
	mUserCount = 0;

	// Start image capture
	startKinect();

	// Start audio capture
	startAudio();

	// Setup the parameters
	mParams = params::InterfaceGl( "Parameters", Vec2i( 245, 500 ) );
	mParams.addText( "DEVICE" );
	mParams.addParam( "Device count", & mDeviceCount, "", true );
	mParams.addParam( "Device index", & mDeviceIndex, "min=0 max=" + toString( math<int32_t>::max( mDeviceCount - 1, 0 ) ) + " step=1" );
	mParams.addParam( "Device angle", & mCameraAngle, "min=-" + toString( Kinect::MAXIMUM_TILT_ANGLE ) + 
		" max=" + toString( Kinect::MAXIMUM_TILT_ANGLE ) + " step=1" );
	mParams.addSeparator();
	mParams.addText( "STATISTICS");
	mParams.addParam( "Collect statistics", & mEnabledStats, "key=t" );
	mParams.addParam( "App frame rate", & mFrameRateApp, "", true );
	mParams.addParam( "Depth frame rate", & mFrameRateDepth, "", true );
	mParams.addParam( "Skeleton frame rate", & mFrameRateSkeletons, "", true );
	mParams.addParam( "Video frame rate", & mFrameRateVideo, "", true );
	mParams.addParam( "User count", & mUserCount, "", true );
	mParams.addSeparator();
	mParams.addText( "CAPTURE" );
	mParams.addParam( "Capture", & mCapture, "key=c" );
	mParams.addParam( "Audio", & mEnabledAudio, "key=a" );
	mParams.addParam( "Depth", & mEnabledDepth, "key=d" );
	mParams.addParam( "Skeletons", & mEnabledSkeletons, "key=k" );
	mParams.addParam( "Video", & mEnabledVideo, "key=v" );
	mParams.addSeparator();
	mParams.addText( "DEPTH IMAGE");
	mParams.addParam( "Remove background", & mRemoveBackground, "key=b" );
	mParams.addParam( "Binary depth mode", & mBinaryMode, "key=w" );
	mParams.addParam( "Invert binary image", & mInverted, "key=i" );
	mParams.addParam( "Near mode", & mEnabledNearMode, "key=n" );
	mParams.addSeparator();
	mParams.addText( "APPLICATION" );
	mParams.addParam( "Full screen", & mFullScreen, "key=f" );
	mParams.addButton( "Screen shot", std::bind(& KinectApp::screenShot, this ), "key=s" );
	mParams.addButton( "Quit", std::bind( & KinectApp::quit, this ), "key=esc" );

}

// Quit
void KinectApp::shutdown()
{

	// Stop audio input
	stopAudio();

	// Stop Kinect input
	for_each ( mDevices.begin(), mDevices.end(), []( KinectRef & device ) {
		device->stop();
	} );

	// Clean up
	mBody.clear();
	if ( mData != 0 ) {
		delete mData;
	}
	mLeftArm.clear();
	mLeftLeg.clear();
	mRightArm.clear();
	mRightLeg.clear();
	mSegments.clear();
	mSkeletons.clear();

}

// Starts audio input
void KinectApp::startAudio()
{

	// Initialize audio device
	if ( !mInput ) {
		mInput = AudioInput::create();
	}

	// Find Kinect audio
	int32_t deviceId = -1;
	int32_t audioDeviceIndex = 0;
	DeviceList devices = mInput->getDeviceList();
	for ( DeviceList::const_iterator deviceIt = devices.cbegin(); deviceIt != devices.cend(); ++deviceIt, audioDeviceIndex ) {
		if ( boost::contains( boost::to_lower_copy( deviceIt->second ), "kinect" ) ) {
			deviceId = deviceIt->first;
			audioDeviceIndex++;
		}
	}
	if ( deviceId >= 0 && deviceId + audioDeviceIndex == mDeviceIndex ) {
		mInput->setDevice( deviceId + mDeviceIndex );
	}

	// Start receiving audio
	mCallbackId = mInput->addCallback<KinectApp>( & KinectApp::onData, this );
	mInput->start();

}

// Start Kinect input
void KinectApp::startKinect()
{

	// Update device count
	mDeviceCount = Kinect::getDeviceCount();

	// Initialize devices
	if ( mDevices.size() <= 0 ) {
		for ( int32_t i = 0; i < mDeviceCount; i++ ) {
			mDevices.push_back( Kinect::create() );
			( * mDevices.rbegin() )->start( i );
		}
	}

	// Configure Kinect
	mDevices[ mDeviceIndex ]->enableBinaryMode( mBinaryMode );
	mDevices[ mDeviceIndex ]->enableDepth( mEnabledDepth );
	mDevices[ mDeviceIndex ]->enableSkeletons( mEnabledSkeletons );
	mDevices[ mDeviceIndex ]->enableVideo( mEnabledVideo );
	mDevices[ mDeviceIndex ]->removeBackground( mRemoveBackground );

	// Kinect camera angle
	mCameraAngle = mDevices[ mDeviceIndex ]->getCameraAngle();
	mCameraAnglePrev = mCameraAngle;

	// Clear stats
	resetStats();

}

// Stops audio indput
void KinectApp::stopAudio()
{

	// Stop input
	mInput->stop();
	mInput->removeCallback( mCallbackId );
	while ( mInput->isReceiving() ) {
	}

}

// Runs update logic
void KinectApp::update()
{

	// Toggle fullscreen
	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
	}

	// Toggle device
	if (mDeviceIndex != mDeviceIndexPrev) {

		// Clamp device index
		mDeviceIndex = math<int32_t>::clamp( mDeviceIndex, 0, mDeviceCount - 1 );
		mDeviceIndexPrev = mDeviceIndex;
		
		// Switch image devices
		if ( mCapture ) {
			startKinect();
		}

	}

	// Toggle background remove
	if ( mRemoveBackground != mRemoveBackgroundPrev ) {
		mDevices[ mDeviceIndex ]->removeBackground( mRemoveBackground );
		mRemoveBackgroundPrev = mRemoveBackground;
	}

	// Toggle capture
	if ( mCapture != mCapturePrev ) {
		mCapturePrev = mCapture;
		if ( mCapture ) {
			mInput->start();
			mDevices[ mDeviceIndex ]->start( mDeviceIndex );
		} else {
			mInput->stop();
			mDevices[ mDeviceIndex ]->stop();
		}
	}

	// Toggle input tracking types
	if ( mEnabledAudio != mEnabledAudioPrev ) {
		mEnabledAudio ? startAudio() : stopAudio();
		mEnabledAudioPrev = mEnabledAudio;
	}
	if ( mEnabledDepth != mEnabledDepthPrev ) {
		mDevices[ mDeviceIndex ]->enableDepth( mEnabledDepth );
		mEnabledDepthPrev = mEnabledDepth;
	}
	if ( mEnabledSkeletons != mEnabledSkeletonsPrev ) {
		mDevices[ mDeviceIndex ]->enableSkeletons( mEnabledSkeletons );
		mEnabledSkeletonsPrev = mEnabledSkeletons;
	}
	if ( mEnabledVideo != mEnabledVideoPrev ) {
		mDevices[ mDeviceIndex ]->enableVideo( mEnabledVideo );
		mEnabledVideoPrev = mEnabledVideo;
	}

	// Toggle binary mode
	if ( mBinaryMode != mBinaryModePrev || mInverted != mInvertedPrev ) {
		mDevices[ mDeviceIndex ]->enableBinaryMode( mBinaryMode, mInverted );
		mBinaryModePrev = mBinaryMode;
		mInvertedPrev = mInverted;
	}

	// Toggle near mode
	if ( mEnabledNearMode != mEnabledNearModePrev ) {
		mDevices[ mDeviceIndex ]->enableNearMode( mEnabledNearMode );
		mEnabledNearModePrev = mEnabledNearMode;
	}

	// Check if device is capturing
	if ( mDevices[ mDeviceIndex ]->isCapturing() ) {

		// Adjust Kinect camera angle, as needed
		if ( mCameraAngle != mCameraAnglePrev ) {
			mDevices[ mDeviceIndex ]->setCameraAngle( mCameraAngle );
			mCameraAnglePrev = mCameraAngle;
		}

		// Get latest Kinect data
		if ( mDevices[ mDeviceIndex ]->checkNewDepthFrame() ) {
			mDepthSurface = mDevices[ mDeviceIndex ]->getDepth();
		}
		if ( mDevices[ mDeviceIndex ]->checkNewSkeletons() ) {
			mSkeletons = mDevices[ mDeviceIndex ]->getSkeletons();
		}
		if ( mDevices[ mDeviceIndex ]->checkNewVideoFrame() ) {
			mVideoSurface = mDevices[ mDeviceIndex ]->getVideo();
		}

		// Statistics enabled (turn off to improve performance)
		if ( mEnabledStats ) {

			// Update user count
			mUserCount = mDevices[ mDeviceIndex ]->getUserCount();
		
			// Update frame rates
			mFrameRateDepth = mDevices[ mDeviceIndex ]->getDepthFrameRate();
			mFrameRateSkeletons = mDevices[ mDeviceIndex ]->getSkeletonsFrameRate();
			mFrameRateVideo = mDevices[ mDeviceIndex ]->getVideoFrameRate();

		} else {

			// Clear stats
			resetStats();

		}

	}

	// Update frame rate
	mFrameRateApp = getAverageFps();

}

// Run application
CINDER_APP_BASIC( KinectApp, RendererGl )

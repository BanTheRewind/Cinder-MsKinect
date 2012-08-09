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
	void								draw();
	void								prepareSettings( ci::app::AppBasic::Settings *settings );
	void								setup();
	void								shutdown();
	void								update();

	// Audio callback
	void 								onAudioData( float *data, int32_t size );

private:

	// Capturing flags
	bool								mCapture;
	bool								mCapturePrev;
	bool								mBinaryMode;
	bool								mBinaryModePrev;
	bool								mEnabledAudio;
	bool								mEnabledAudioPrev;
	bool								mEnabledDepth;
	bool								mEnabledDepthPrev;
	bool								mEnabledNearMode;
	bool								mEnabledNearModePrev;
	bool								mEnabledSeatedMode;
	bool								mEnabledSeatedModePrev;
	bool								mEnabledSkeletons;
	bool								mEnabledSkeletonsPrev;
	bool								mEnabledStats;
	bool								mEnabledVideo;
	bool								mEnabledVideoPrev;
	bool								mFlipped;
	bool								mFlippedPrev;
	bool								mInverted;
	bool								mInvertedPrev;

	// Audio input
	int32_t								mCallbackId;
	float *								mData;
	AudioInputRef						mInput;

	// Kinect
	ci::Surface16u						mDepthSurface;
	int32_t								mDeviceCount;
	KinectSdk::DeviceOptions			mDeviceOptions;
	KinectSdk::KinectRef				mKinect;
	std::vector<KinectSdk::Skeleton>	mSkeletons;
	int32_t								mTilt;
	int32_t								mTiltPrev;
	int32_t								mUserCount;
	ci::Surface8u						mVideoSurface;
	void								startAudio();
	void								startKinect();
	void								stopAudio();

	// Kinect callbacks
	int32_t								mCallbackDepthId;
	int32_t								mCallbackSkeletonId;
	int32_t								mCallbackVideoId;
	void								onDepthData( ci::Surface16u surface, const KinectSdk::DeviceOptions &deviceOptions );
	void								onSkeletonAudioData( std::vector<KinectSdk::Skeleton> skeletons, const KinectSdk::DeviceOptions &deviceOptions );
	void								onVideoData( ci::Surface8u surface, const KinectSdk::DeviceOptions &deviceOptions );

	// Camera
	ci::CameraPersp						mCamera;

	// Params
	float								mFrameRateApp;
	float								mFrameRateDepth;
	float								mFrameRateSkeletons;
	float								mFrameRateVideo;
	bool								mFullScreen;
	ci::params::InterfaceGl				mParams;
	bool								mRemoveBackground;
	bool								mRemoveBackgroundPrev;
	void								resetStats();

	// Save screen shot
	void								screenShot();

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// Render
void KinectApp::draw()
{

	// Clear window
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::gray( 0.1f ) );

	// We're capturing
	if ( mKinect->isCapturing() ) {

		// Set up camera for 3D
		gl::setMatrices( mCamera );

		// Move skeletons down below the rest of the interface
		gl::pushMatrices();
		gl::scale( Vec2f::one() * 0.5f );
		gl::translate( 0.0f, -0.62f, 0.0f );

		// Iterate through skeletons
		uint32_t i = 0;
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.cbegin(); skeletonIt != mSkeletons.cend(); ++skeletonIt, i++ ) {

			// Set color
			gl::color( mKinect->getUserColor( i ) );

			// Draw bones and joints
			for ( Skeleton::const_iterator boneIt = skeletonIt->cbegin(); boneIt != skeletonIt->cend(); ++boneIt ) {
					
				// Get positions of each joint in this bone to draw it
				const Bone &bone	= boneIt->second;
				Vec3f position		= bone.getPosition();
				Vec3f destination	= skeletonIt->at( bone.getStartJoint() ).getPosition();

				// Draw bone
				gl::drawLine( position, destination );

				// Draw joint
				gl::drawSphere( position, 0.025f, 16 );

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
			gl::draw( gl::Texture( mVideoSurface ), srcArea, destRect );
		}

	}

	// Check audio data
	if ( mData != 0 ) {

		// Get dimensions
		int32_t dataSize	= mInput->getDataSize();
		float scale			= 240.0f / (float)dataSize;
		float height		= 180.0f;
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
		if ( mLine.size() > 0 ) {
			gl::draw( mLine );
		}

	}

	// Draw the interface
	params::InterfaceGl::draw();

}

// Called when audio buffer is full
void KinectApp::onAudioData( float *data, int32_t size )
{
	mData = data;
}

// Receives depth data
void KinectApp::onDepthData( Surface16u surface, const DeviceOptions &deviceOptions )
{
	mDepthSurface = surface;
}

// Receives skeleton data
void KinectApp::onSkeletonAudioData( vector<Skeleton> skeletons, const DeviceOptions &deviceOptions )
{
	mSkeletons = skeletons;
}

// Receives video data
void KinectApp::onVideoData( Surface8u surface, const DeviceOptions &deviceOptions )
{
	mVideoSurface = surface;
}

// Prepare window
void KinectApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1005, 570 );
	settings->setFrameRate( 60.0f );
}

// Reset statistics
void KinectApp::resetStats()
{
	mFrameRateDepth		= 0.0f;
	mFrameRateSkeletons	= 0.0f;
	mFrameRateVideo		= 0.0f;
	mUserCount			= 0;
}

// Take screen shot
void KinectApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void KinectApp::setup()
{

	// Set up OpenGL
	glLineWidth( 2.0f );
	gl::color( ColorAf::white() );

	// Set up camera
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 3.0f ), Vec3f::zero() );
	mCamera.setPerspective( 45.0f, getWindowAspectRatio(), 1.0f, 1000.0f );

	// Initialize parameters
	mBinaryMode				= false;
	mBinaryModePrev			= mBinaryMode;
	mCapture				= true;
	mCapturePrev			= mCapture;
	mDeviceCount			= 0;
	mEnabledAudio			= true;
	mEnabledAudioPrev		= true;
	mEnabledDepth			= true;
	mEnabledDepthPrev		= mEnabledDepth;
	mEnabledNearMode		= false;
	mEnabledNearModePrev	= mEnabledNearMode;
	mEnabledSeatedMode		= false;
	mEnabledSeatedModePrev	= mEnabledSeatedMode;
	mEnabledSkeletons		= true;
	mEnabledSkeletonsPrev	= mEnabledSkeletons;
	mEnabledStats			= true;
	mEnabledVideo			= true;
	mEnabledVideoPrev		= mEnabledVideo;
	mFlipped				= false;
	mFlippedPrev			= mFlipped;
	mFrameRateApp			= 0.0f;
	mFrameRateDepth			= 0.0f;
	mFrameRateSkeletons		= 0.0f;
	mFrameRateVideo			= 0.0f;
	mFullScreen				= isFullScreen();
	mInverted				= false;
	mInvertedPrev			= mInverted;
	mRemoveBackground		= false;
	mRemoveBackgroundPrev	= mRemoveBackground;
	mUserCount				= 0;

	// Start image capture
	mKinect = Kinect::create();
	startKinect();

	// Add callbacks
	mCallbackDepthId	= mKinect->addDepthCallback( &KinectApp::onDepthData, this );
	mCallbackSkeletonId	= mKinect->addSkeletonTrackingCallback( &KinectApp::onSkeletonAudioData, this );
	mCallbackVideoId	= mKinect->addVideoCallback( &KinectApp::onVideoData, this );

	// Start audio capture
	startAudio();

	// Setup the parameters
	mParams = params::InterfaceGl( "Parameters", Vec2i( 245, 500 ) );
	mParams.addText( "DEVICE" );
	mParams.addParam( "Device count",			&mDeviceCount,							"", true				);
	mParams.addParam( "Device angle",			&mTilt,									"min=-" + 
		toString( Kinect::MAXIMUM_TILT_ANGLE ) + " max=" + toString( Kinect::MAXIMUM_TILT_ANGLE ) + " step=1"	);
	mParams.addSeparator();
	mParams.addText( "STATISTICS");
	mParams.addParam( "Collect statistics",		&mEnabledStats,							"key=t"					);
	mParams.addParam( "App frame rate",			&mFrameRateApp,							"", true				);
	mParams.addParam( "Depth frame rate",		&mFrameRateDepth,						"", true				);
	mParams.addParam( "Skeleton frame rate",	&mFrameRateSkeletons,					"", true				);
	mParams.addParam( "Video frame rate",		&mFrameRateVideo,						"", true				);
	mParams.addParam( "User count",				&mUserCount,							"", true				);
	mParams.addSeparator();
	mParams.addText( "CAPTURE" );
	mParams.addParam( "Capture",				&mCapture,								"key=c" 				);
	mParams.addParam( "Audio",					&mEnabledAudio,							"key=a" 				);
	mParams.addParam( "Depth",					&mEnabledDepth,							"key=d" 				);
	mParams.addParam( "Skeletons",				&mEnabledSkeletons,						"key=k" 				);
	mParams.addParam( "Video",					&mEnabledVideo,							"key=v" 				);
	mParams.addSeparator();
	mParams.addText( "INPUT");
	mParams.addParam( "Remove background",		&mRemoveBackground,						"key=b" 				);
	mParams.addParam( "Binary depth mode",		&mBinaryMode,							"key=w" 				);
	mParams.addParam( "Invert binary image",	&mInverted,								"key=i" 				);
	mParams.addParam( "Flip input",				&mFlipped,								"key=m" 				);
	mParams.addParam( "Near mode",				&mEnabledNearMode,						"key=n" 				);
	mParams.addParam( "Seated mode",			&mEnabledSeatedMode,					"key=e" 				);
	mParams.addSeparator();
	mParams.addText( "APPLICATION" );
	mParams.addParam( "Full screen",			&mFullScreen,							"key=f"					);
	mParams.addButton( "Screen shot",			bind( &KinectApp::screenShot, this ),	"key=s"					);
	mParams.addButton( "Quit",					bind( &KinectApp::quit, this ),			"key=esc"				);

}

// Quit
void KinectApp::shutdown()
{

	// Stop input
	stopAudio();
	mKinect->removeCallback( mCallbackDepthId );
	mKinect->removeCallback( mCallbackSkeletonId );
	mKinect->removeCallback( mCallbackVideoId );
	mKinect->stop();

	// Clean up
	if ( mData != 0 ) {
		delete mData;
	}
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

	// Start receiving audio
	mInput->setDevice( deviceId );
	mCallbackId = mInput->addCallback( & KinectApp::onAudioData, this );
	mInput->start();

}

// Start Kinect input
void KinectApp::startKinect()
{

	// Update device count
	mDeviceCount = Kinect::getDeviceCount();
	
	// Configure device
	mDeviceOptions.enableDepth( mEnabledDepth );
	mDeviceOptions.enableNearMode( mEnabledNearMode );
	mDeviceOptions.enableSkeletonTracking( mEnabledSkeletons, mEnabledSeatedMode );
	mDeviceOptions.enableVideo( mEnabledVideo );
	mKinect->enableBinaryMode( mBinaryMode );
	mKinect->removeBackground( mRemoveBackground );
	mKinect->setFlipped( mFlipped );

	// Stop, if capturing
	if ( mKinect->isCapturing() ) {
		mKinect->stop();
	}

	// Start Kinect
	mKinect->start( mDeviceOptions );
	
	// Trace out the unique device ID
	console() << "Device ID: " << mKinect->getDeviceOptions().getDeviceId() << endl;

	// Get device angle angle
	mTilt = mKinect->getTilt();
	mTiltPrev = mTilt;

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

	// Update frame rate
	mFrameRateApp = getAverageFps();

	// Toggle fullscreen
	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
	}

	// Toggle background remove
	if ( mRemoveBackground != mRemoveBackgroundPrev ) {
		mKinect->removeBackground( mRemoveBackground );
		mRemoveBackgroundPrev = mRemoveBackground;
	}

	// Toggle mirror image
	if ( mFlipped != mFlippedPrev ) {
		mKinect->setFlipped( mFlipped );
		mFlippedPrev = mFlipped;
	}

	// Toggle capture
	if ( mCapture != mCapturePrev ) {
		mCapturePrev = mCapture;
		if ( mCapture ) {
			mInput->start();
			startKinect();
		} else {
			mInput->stop();
			mKinect->stop();
		}
	}

	// Toggle audio
	if ( mEnabledAudio != mEnabledAudioPrev ) {
		mEnabledAudio ? startAudio() : stopAudio();
		mEnabledAudioPrev = mEnabledAudio;
	}

	// Toggle input tracking types (requires device restart)
	if ( mEnabledDepth		!= mEnabledDepthPrev		|| 
		mEnabledNearMode	!= mEnabledNearModePrev		|| 
		mEnabledSeatedMode	!= mEnabledSeatedModePrev	|| 
		mEnabledSkeletons	!= mEnabledSkeletonsPrev	|| 
		mEnabledVideo		!= mEnabledVideoPrev ) {
		startKinect();
		mEnabledDepthPrev		= mEnabledDepth;
		mEnabledNearModePrev	= mEnabledNearMode;
		mEnabledSeatedModePrev	= mEnabledSeatedMode;
		mEnabledSkeletonsPrev	= mEnabledSkeletons;
		mEnabledVideoPrev		= mEnabledVideo;
	}

	// Toggle binary mode
	if ( mBinaryMode	!= mBinaryModePrev	|| 
		mInverted		!= mInvertedPrev ) {
		mKinect->enableBinaryMode( mBinaryMode, mInverted );
		mBinaryModePrev	= mBinaryMode;
		mInvertedPrev	= mInverted;
	}

	// Check if device is capturing
	if ( mKinect->isCapturing() ) {

		// Update device
		mKinect->update();

		// Adjust Kinect camera angle, as needed
		if ( mTilt != mTiltPrev ) {
			mKinect->setTilt( mTilt );
			mTiltPrev = mTilt;
		}

		// Statistics enabled (turn off to improve performance)
		if ( mEnabledStats ) {

			// Update user count
			mUserCount			= mKinect->getUserCount();
		
			// Update frame rates
			mFrameRateDepth		= mKinect->getDepthFrameRate();
			mFrameRateSkeletons	= mKinect->getSkeletonFrameRate();
			mFrameRateVideo		= mKinect->getVideoFrameRate();

		} else {

			// Clear stats
			resetStats();

		}

	} else {

		// If Kinect initialization failed, try again every 90 frames
		if ( getElapsedFrames() % 90 == 0 ) {
			mKinect->start();
		}

	}

}

// Run application
CINDER_APP_BASIC( KinectApp, RendererGl )

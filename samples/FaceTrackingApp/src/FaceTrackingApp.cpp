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
#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"
#include "cinder/Utilities.h"
#include "FaceTracker.h"
#include "Kinect.h"

/* 
* This application demonstrates how to the Face Tracker SDK with the 
* Kinect SDK to track faces with a Kinect.
*/ 
class FaceTrackingApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void keyDown( ci::app::KeyEvent event );
	void prepareSettings( ci::app::AppBasic::Settings *settings );
	void setup();
	void shutdown();
	void update();

private:

	// Kinect
	KinectSdk::KinectRef				mKinect;
	std::vector<KinectSdk::Skeleton>	mSkeletons;
	ci::Surface16u						mSurfaceDepth;
	ci::Surface8u						mSurfaceVideo;

	// Kinect callbacks
	int32_t								mCallbackDepthId;
	int32_t								mCallbackSkeletonsId;
	int32_t								mCallbackVideoId;
	void								onDepthData( ci::Surface16u surface, const KinectSdk::DeviceOptions &deviceOptions );
	void								onSkeletonData( std::vector<KinectSdk::Skeleton> skeletons, const KinectSdk::DeviceOptions &deviceOptions );
	void								onVideoData( ci::Surface8u surface, const KinectSdk::DeviceOptions &deviceOptions );

	// Face tracker
	FaceTrackerRef						mFaceTracker;

	// Save screenshot
	void								screenShot();

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// Render
void FaceTrackingApp::draw()
{

	// Clear window
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::gray( 0.1f ) );
	gl::setMatricesWindow( getWindowSize() );

	// Draw streams
	if ( mSurfaceVideo ) {
		gl::draw( gl::Texture( mSurfaceVideo ), Rectf( 0.0f, 0.0f, 320.0f, 240.0f ) );
		if ( mSurfaceDepth ) {
			gl::draw( gl::Texture( mSurfaceDepth ), Rectf( 320.0f, 0.0f, 640.0f, 240.0f ) );
		}
	}

}

// Handles key press
void FaceTrackingApp::keyDown( KeyEvent event )
{

	// Quit, toggle fullscreen
	switch ( event.getCode() ) {
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_f:
		setFullScreen( !isFullScreen() );
		break;
	case KeyEvent::KEY_SPACE:
		screenShot();
		break;
	}

}

// Receives depth data
void FaceTrackingApp::onDepthData( Surface16u surface, const DeviceOptions &deviceOptions )
{
	mSurfaceDepth = surface;
}

// Receives skeleton data
void FaceTrackingApp::onSkeletonData( vector<Skeleton> skeletons, const DeviceOptions &deviceOptions )
{
	mSkeletons = skeletons;
}

// Receives video data
void FaceTrackingApp::onVideoData( Surface8u surface, const DeviceOptions &deviceOptions )
{
	mSurfaceVideo = surface;
}

// Prepare window
void FaceTrackingApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
}

// Take screen shot
void FaceTrackingApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void FaceTrackingApp::setup()
{

	// Start Kinect
	mKinect = Kinect::create();
	DeviceOptions deviceOptions;
	mKinect->start( deviceOptions );
	mKinect->removeBackground();
	mCallbackDepthId = mKinect->addSkeletonTrackingCallback<FaceTrackingApp>( &FaceTrackingApp::onSkeletonData, this );
	mCallbackSkeletonsId = mKinect->addSkeletonTrackingCallback<FaceTrackingApp>( &FaceTrackingApp::onSkeletonData, this );
	mCallbackVideoId = mKinect->addSkeletonTrackingCallback<FaceTrackingApp>( &FaceTrackingApp::onSkeletonData, this );

	// Set up face tracker
	mFaceTracker = FaceTracker::create();
	mFaceTracker->setup( deviceOptions );

	// Set the skeleton smoothing to remove jitters. Better smoothing means
	// less jitters, but a slower response time.nclude
	mKinect->setTransform( Kinect::TRANSFORM_SMOOTH );

}

// Called on exit
void FaceTrackingApp::shutdown()
{

	// Stop input
	mKinect->removeCallback( mCallbackDepthId );
	mKinect->removeCallback( mCallbackSkeletonsId );
	mKinect->removeCallback( mCallbackVideoId );
	mKinect->stop();

}

// Runs update logic
void FaceTrackingApp::update()
{

	// Kinect is capturing
	if ( mKinect->isCapturing() ) {
		mKinect->update();
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.cbegin(); skeletonIt != mSkeletons.cend(); ++skeletonIt ) {
			if ( mFaceTracker->findFace( *skeletonIt, mSurfaceVideo, mSurfaceDepth ) ) {
				break;
			}
		}
	} else {

		// If Kinect initialization failed, try again every 90 frames
		if ( getElapsedFrames() % 90 == 0 ) {
			mKinect->start();
		}

	}

}

// Run application
CINDER_APP_BASIC( FaceTrackingApp, RendererGl )

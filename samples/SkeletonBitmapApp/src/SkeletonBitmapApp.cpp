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
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"
#include "cinder/Utilities.h"
#include "Kinect.h"

/* 
* This application demonstrates how to acquire and display 
* skeleton data.
*/ 
class SkeletonBitmapApp : public ci::app::AppBasic 
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
	ci::gl::Texture						mTexture;

	void screenShot();

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// Render
void SkeletonBitmapApp::draw()
{

	// Clear window
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf( 0.1f, 0.1f, 0.1f ) );
	gl::setMatricesWindow( getWindowSize() );

	// We're capturing
	if ( mKinect->isCapturing() && mTexture ) {
		
		// Draw color image
		gl::color( ColorAf::white() );
		gl::draw( mTexture, getWindowBounds() );
		
		// Scale skeleton to fit
		gl::pushMatrices();
		gl::scale( Vec2f( getWindowSize() ) / Vec2f( mTexture.getSize() ) );

		// Iterate through skeletons
		uint32_t i = 0;
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.cbegin(); skeletonIt != mSkeletons.cend(); ++skeletonIt, i++ ) {

			// Valid skeletons have all joints
			if ( skeletonIt->size() == JointName::NUI_SKELETON_POSITION_COUNT ) {

				// Set color
				gl::color( mKinect->getUserColor( i ) );

				// Draw bones and joints
				for ( Skeleton::const_iterator boneIt = skeletonIt->cbegin(); boneIt != skeletonIt->cend(); ++boneIt ) {
					
					// Get joint positions 
					Vec3f position			= boneIt->second.getPosition();
					Vec3f destination		= skeletonIt->at( boneIt->second.getStartJoint() ).getPosition();
					Vec2i positionScreen	= mKinect->getSkeletonVideoPos( position );
					Vec2i destinationSceen	= mKinect->getSkeletonVideoPos( destination );

					// Draw bone
					gl::begin( GL_LINES );
					gl::vertex( positionScreen );
					gl::vertex( destinationSceen );
					glEnd();

					// Draw joint
					gl::drawSolidCircle( positionScreen, 10.0f, 16 );

				}

			}

		}

		gl::popMatrices();

	}

}

// Handles key press
void SkeletonBitmapApp::keyDown( KeyEvent event )
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

// Prepare window
void SkeletonBitmapApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
}

// Take screen shot
void SkeletonBitmapApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void SkeletonBitmapApp::setup()
{

	// Set up OpenGL
	glLineWidth( 2.0f );
	gl::color( ColorAf::white() );

	// Start Kinect
	mKinect = Kinect::create();
	mKinect->start( DeviceOptions().enableDepth( false ) );

}

// Called on exit
void SkeletonBitmapApp::shutdown()
{

	// Stop input
	mKinect->stop();

	// Clean up
	mSkeletons.clear();

}

// Runs update logic
void SkeletonBitmapApp::update()
{

	// Kinect is capturing
	if ( mKinect->isCapturing() ) {
	
		// Update video
		if ( mKinect->checkNewVideoFrame() ) {
			mTexture = gl::Texture( mKinect->getVideo() );
		}

		// Acquire skeletons
		if ( mKinect->checkNewSkeletons() ) {
			mSkeletons = mKinect->getSkeletons();
		}

	} else {

		// If Kinect initialization failed, try again every 90 frames
		if ( getElapsedFrames() % 90 == 0 ) {
			mKinect->start();
		}

	}

}

// Run application
CINDER_APP_BASIC( SkeletonBitmapApp, RendererGl )

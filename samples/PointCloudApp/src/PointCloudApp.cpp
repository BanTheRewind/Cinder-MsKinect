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
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "Kinect.h"

/* 
* This application demonstrates how to represent the 
* Kinect's depth image in 3D space.
*/
class PointCloudApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void keyDown( ci::app::KeyEvent event );
	void prepareSettings( ci::app::AppBasic::Settings * settings );
	void shutdown();
	void setup();
	void update();

private:

	// Kinect
	ci::Surface8u			mSurface;
	KinectSdk::KinectRef	mKinect;

	// Depth points
	std::vector<ci::Vec3f>	mPoints;
	ci::Vec3f				mOffset;
	float					mScale;
	ci::Vec3f				mSpacing;

	// Camera
	ci::CameraPersp			mCamera;

	// Save screen shot
	void					screenShot();

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// Render
void PointCloudApp::draw()
{

	// Clear window
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::black() );
	gl::setMatrices( mCamera );

	// Draw points
	gl::pushMatrices();
	gl::scale( Vec3f( mScale, mScale, 1.0f ) );
	gl::translate( mOffset );
	glBegin( GL_POINTS );
	for_each(mPoints.cbegin(), mPoints.cend(), [](const Vec3f & point) {
		gl::vertex( point );
	} );
	glEnd();
	gl::popMatrices();

}

// Handles key press
void PointCloudApp::keyDown( KeyEvent event )
{

	// Key on key...
	switch ( event.getCode() ) {
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_f:
		setFullScreen( !isFullScreen() );
		break;
	case KeyEvent::KEY_s:
		screenShot();
		break;
	}

}

// Prepare window
void PointCloudApp::prepareSettings( Settings * settings )
{

	// DO IT!
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate( 60.0f );

}

// Quit
void PointCloudApp::shutdown()
{

	// Stop input
	mKinect->stop();

	// Clean up
	mPoints.clear();

}

// Take screen shot
void PointCloudApp::screenShot()
{

	// DO IT!
	writeImage( getAppPath() + "frame" + toString( getElapsedFrames() ) + ".png", copyWindowSurface() );

}

// Set up
void PointCloudApp::setup()
{

	// Set up OpenGL
	gl::enable( GL_BLEND );
	gl::enable( GL_DEPTH_TEST );
	glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
	glEnable( GL_POINT_SMOOTH );
	glPointSize( 0.5f );
	gl::enableAlphaBlending();
	gl::color( ColorAf::white() );

	// Start Kinect with isolated depth tracking only
	mKinect = Kinect::create();
	mKinect->removeBackground();
	mKinect->enableSkeletons( false );
	mKinect->enableVideo( false );
	mKinect->start( 0 );

	// Set up camera
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 0.001f ), Vec3f::zero() );
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 1.0f, 1000.0f );

	// Point scaling
	mScale = 4.0f;
	mSpacing.set( 1.0f / 640.0f, -1.0f / 480.0f, 1.0f / 255.0f );
	mOffset.set( -0.25f, 0.25f, 0.0f );

}

// Runs update logic
void PointCloudApp::update()
{

	// Device is capturing
	if ( mKinect->isCapturing() ) {

		// Check for latest depth map
		if ( mKinect->checkNewDepthFrame() ) {

			// Get surface
			mSurface = mKinect->getDepth();

			// Clear point list
			Vec3f position = Vec3f::zero();
			mPoints.clear();

			// Iterate image rows
			Surface::Iter iter = mSurface.getIter();
			while ( iter.line() ) {

				// Iterate rows in pixel
				while (iter.pixel()) {

					// Get channels
					uint8_t b = iter.b();
					uint8_t g = iter.g();
					uint8_t r = iter.r();

					// This is not black
					if ( b + g + r > 0 ) {

						// Choose which channel to use for depth
						uint8_t depth = b;
						if ( g > depth ) {
							depth = g;
						}
						if ( r > depth ) {
							depth = r;
						}

						// Invert depth
						depth = 256 - depth;

						// Add position to point list
						position.z = -mSpacing.z * ( (float)depth * 5.0f );
						mPoints.push_back( position );

					}

					// Shift point
					position.x += mSpacing.x;

				}

				// Update position
				position.x = 0.0f;
				position.y += mSpacing.y;

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
CINDER_APP_BASIC( PointCloudApp, RendererGl )

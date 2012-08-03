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
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "Kinect.h"

/* 
* This application demonstrates how to run multiple
* Kinect devices simultaneously.  Note that each device 
* uses 61% of the bandwidth of a USB 2.0 controller. If 
* this application recognizes the device but does not display
* its image, then connect the device to an addition USB
* controller.
*/
class MultiDeviceApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void								draw();
	void								keyDown( ci::app::KeyEvent event );
	void								prepareSettings( ci::app::AppBasic::Settings *settings );
	void								shutdown();
	void								setup();
	void								update();

private:

	// Device info
	struct Device
	{
		int32_t					mCallbackId;
		KinectSdk::KinectRef	mKinect;
		ci::gl::Texture			mTexture;
	};
	std::vector<Device>			mDevices;

	// Callback
	void						onDepthData( ci::Surface16u surface, const KinectSdk::DeviceOptions &deviceOptions );

	// Save screen shot
	void						screenShot();

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// Render
void MultiDeviceApp::draw()
{

	// Clear window
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::black() );
	
	// Draw images
	gl::color( ColorAf::white() );
	int32_t width	= getWindowWidth() / mDevices.size();
	int32_t height	= ( width * 3 ) / 4;
	int32_t y		= ( getWindowHeight() - height ) / 2;
	for ( uint32_t i = 0; i < mDevices.size(); ++i ) {
		if ( mDevices.at( i ).mTexture ) {
			gl::draw( mDevices.at( i ).mTexture, Area( width * i, y, width * ( i + 1 ), y + height ) );
		}
	}

}

// Handles key press
void MultiDeviceApp::keyDown( KeyEvent event )
{
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

void MultiDeviceApp::onDepthData( ci::Surface16u surface, const KinectSdk::DeviceOptions &deviceOptions )
{
	int32_t index = deviceOptions.getDeviceIndex();
	for ( size_t i = 0; i < mDevices.size(); ++i ) {
		if ( index == mDevices.at( i ).mKinect->getDeviceOptions().getDeviceIndex() ) {
			mDevices.at( i ).mTexture = gl::Texture( surface );
			break;
		}
	}
}

// Prepare window
void MultiDeviceApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate( 60.0f );
}

// Take screen shot
void MultiDeviceApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void MultiDeviceApp::setup()
{

	// Set up OpenGL
	gl::enable( GL_DEPTH_TEST );
	gl::enableAlphaBlending();
	gl::enableAdditiveBlending();
	gl::color( ColorAf( Colorf::white(), 0.667f ) );

	DeviceOptions deviceOptions;
	deviceOptions.enableSkeletonTracking( false );
	deviceOptions.enableUserTracking( false );
	deviceOptions.enableVideo( false );

	// Start all available devices
	int32_t count = Kinect::getDeviceCount();
	for ( int32_t i = 0; i < count; i++ ) {
		deviceOptions.setDeviceIndex( i );
		
		Device device;
		device.mKinect = Kinect::create();
		device.mKinect->start( deviceOptions );
		device.mCallbackId = device.mKinect->addDepthCallback( &MultiDeviceApp::onDepthData, this );
		device.mTexture = gl::Texture( 320, 240 );
		
		mDevices.push_back( device );

		console() << device.mKinect->getDeviceOptions().getDeviceIndex() << ": " << device.mKinect->getDeviceOptions().getDeviceId() << endl;
	}

}

// Called on exit
void MultiDeviceApp::shutdown()
{
	for ( uint32_t i = 0; i < mDevices.size(); i++ ) {
		Device& device = mDevices.at( i );
		device.mKinect->removeCallback( device.mCallbackId );
		device.mKinect->stop();
	}
	mDevices.clear();
}

// Runs update logic
void MultiDeviceApp::update()
{

	// Update Kinect
	for ( uint32_t i = 0; i < mDevices.size(); i++ ) {
		Device& device = mDevices.at( i );
		if ( device.mKinect->isCapturing() ) {
			device.mKinect->update();
		}
	}

}

// Run application
CINDER_APP_BASIC( MultiDeviceApp, RendererGl )

/*
* 
* Copyright (c) 2013, Ban the Rewind
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
	void						draw();
	void						keyDown( ci::app::KeyEvent event );
	void						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						shutdown();
	void						setup();
	void						update();
private:
	struct Device
	{
		MsKinect::DeviceRef		mDevice;
		ci::gl::TextureRef		mTexture;
	};
	std::vector<Device>			mDevices;

	void						onFrame( MsKinect::Frame frame, const MsKinect::DeviceOptions &deviceOptions );

	void						screenShot();
};

using namespace ci;
using namespace ci::app;
using namespace MsKinect;
using namespace std;

void MultiDeviceApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::black() );
	
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

void MultiDeviceApp::keyDown( KeyEvent event )
{
	switch ( event.getCode() ) {
	case KeyEvent::KEY_q:
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

void MultiDeviceApp::onFrame( Frame frame, const MsKinect::DeviceOptions &deviceOptions )
{
	int32_t index = deviceOptions.getDeviceIndex();
	for ( size_t i = 0; i < mDevices.size(); ++i ) {
		if ( index == mDevices.at( i ).mDevice->getDeviceOptions().getDeviceIndex() ) {
			mDevices.at( i ).mTexture = gl::Texture::create( frame.getDepthSurface() );
			break;
		}
	}
}

void MultiDeviceApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate( 60.0f );
}

void MultiDeviceApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void MultiDeviceApp::setup()
{
	gl::enable( GL_DEPTH_TEST );
	gl::enableAlphaBlending();
	gl::enableAdditiveBlending();
	gl::color( ColorAf( Colorf::white(), 0.667f ) );

	DeviceOptions deviceOptions;
	deviceOptions.enableSkeletonTracking( false );
	deviceOptions.enableUserTracking( false );
	deviceOptions.enableColor( false );

	int32_t count = MsKinect::Device::getDeviceCount();
	for ( int32_t i = 0; i < count; i++ ) {
		deviceOptions.setDeviceIndex( i );
		
		Device device;
		device.mDevice = MsKinect::Device::create();
		device.mDevice->start( deviceOptions );
		device.mDevice->connectEventHandler( &MultiDeviceApp::onFrame, this );
		device.mTexture = gl::Texture::create( 320, 240 );
		
		mDevices.push_back( device );

		console() << device.mDevice->getDeviceOptions().getDeviceIndex() << ": " << device.mDevice->getDeviceOptions().getDeviceId() << endl;
	}
}

void MultiDeviceApp::shutdown()
{
	for ( uint32_t i = 0; i < mDevices.size(); ++i ) {
		Device& device = mDevices.at( i );
		device.mDevice->stop();
	}
	mDevices.clear();
}

void MultiDeviceApp::update()
{
	for ( uint32_t i = 0; i < mDevices.size(); ++i ) {
		Device& device = mDevices.at( i );
		if ( device.mDevice->isCapturing() ) {
			device.mDevice->update();
		}
	}

}

CINDER_APP_BASIC( MultiDeviceApp, RendererGl )

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
#include "cinder/Vector.h"
#include "FaceTracker.h"

/* 
* This application demonstrates how to implement the 
* Face Tracking API.
*/
class FaceTrackingApp : public ci::app::AppBasic 
{

public:
	void							draw();
	void							keyDown( ci::app::KeyEvent event );
	void							prepareSettings( ci::app::AppBasic::Settings* settings );
	void							setup();
	void							shutdown();
	void							update();
private:
	ci::Channel16u					mChannelDepth;
	MsKinect::FaceTracker::Face		mFace;
	MsKinect::DeviceRef				mDevice;
	std::vector<MsKinect::Skeleton>	mSkeletons;
	ci::Surface8u					mSurfaceColor;
	
	void							onFace( MsKinect::FaceTracker::Face face );
	void							onFrame( MsKinect::Frame frame, const MsKinect::DeviceOptions& deviceOptions );

	double							mFaceTrackedTime;
	MsKinect::FaceTrackerRef		mFaceTracker;

	void							screenShot();
};

#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace MsKinect;
using namespace std;

void FaceTrackingApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::black() );
	gl::setMatricesWindow( getWindowSize() );

	gl::color( ColorAf::white() );
	if ( mSurfaceColor ) {
		gl::draw( gl::Texture::create( mSurfaceColor ) );
	}
	if ( getElapsedSeconds() - mFaceTrackedTime < 1.0 ) {
		if ( mFace.getMesh2d().getNumIndices() > 0 ) {
			gl::enableWireframe();
			gl::draw( mFace.getMesh2d() );
			gl::disableWireframe();
		}
		gl::drawStrokedRect( mFace.getBounds() );
	}
}

void FaceTrackingApp::keyDown( KeyEvent event )
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

void FaceTrackingApp::onFrame( Frame frame, const DeviceOptions& deviceOptions )
{
	mChannelDepth	= frame.getDepthSurface().getChannelRed();
	mSkeletons		= frame.getSkeletons();
	mSurfaceColor	= frame.getColorSurface();
}

void FaceTrackingApp::onFace( FaceTracker::Face face )
{
	long hr = mFaceTracker->getResult()->GetStatus();
	if ( hr == S_OK ) {
		mFace				= face;
		mFaceTrackedTime	= getElapsedSeconds();

		console() << "Start: " << mFaceTracker->mStartCount << " / Continue: " << mFaceTracker->mContinueCount << endl;
	}
}

void FaceTrackingApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 640, 480 );
	settings->setFrameRate( 60.0f );
}

void FaceTrackingApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void FaceTrackingApp::setup()
{
	// Face tracker expects BGRA color image.
	DeviceOptions deviceOptions;
	deviceOptions.setColorSurfaceChannelOrder( SurfaceChannelOrder::BGRA );

	mDevice = Device::create();
	mDevice->connectEventHandler( &FaceTrackingApp::onFrame, this );
	mDevice->start( deviceOptions );

	mFaceTracker = FaceTracker::create();
	mFaceTracker->enableCalcMesh( false );
	mFaceTracker->connectEventHander( &FaceTrackingApp::onFace, this );
	try {
		mFaceTracker->start( mDevice->getDeviceOptions() );
	} catch ( FaceTracker::ExcFaceTrackerCreate ex ) {
		console() << ex.what() << endl;
		quit();
	} catch ( FaceTracker::ExcFaceTrackerCreateResult ex ) {
		console() << ex.what() << endl;
		quit();
	} catch ( FaceTracker::ExcFaceTrackerInit ex ) {
		console() << ex.what() << endl;
		quit();
	}
}

void FaceTrackingApp::shutdown()
{
	mDevice->stop();
}

void FaceTrackingApp::update()
{
	if ( mDevice->isCapturing() ) {
		mDevice->update();
		if ( mSurfaceColor && mChannelDepth ) {
			mFaceTracker->update( mSurfaceColor, mChannelDepth );
		}
	} else {
		if ( getElapsedFrames() % 90 == 0 ) {
			mDevice->start();
		}
	}
}

CINDER_APP_BASIC( FaceTrackingApp, RendererGl )

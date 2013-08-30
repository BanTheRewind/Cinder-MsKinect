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
#include "cinder/gl/Texture.h"
#include "Kinect.h"

/* 
* This application demonstrates how to acquire and display 
* skeleton data.
*/ 
class SkeletonBitmapApp : public ci::app::AppBasic 
{
public:
	void draw();
	void keyDown( ci::app::KeyEvent event );
	void prepareSettings( ci::app::AppBasic::Settings* settings );
	void setup();
	void shutdown();
	void update();
private:
	KinectSdk::KinectRef				mKinect;
	std::vector<KinectSdk::Skeleton>	mSkeletons;
	ci::gl::TextureRef					mTexture;

	void								onFrame( KinectSdk::Frame frame, const KinectSdk::DeviceOptions &deviceOptions );

	void screenShot();
};

#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

void SkeletonBitmapApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );

	if ( mKinect->isCapturing() && mTexture ) {
		gl::color( ColorAf::white() );
		gl::draw( mTexture, getWindowBounds() );
		
		// Scale skeleton to fit
		gl::pushMatrices();
		gl::scale( Vec2f( getWindowSize() ) / Vec2f( mTexture->getSize() ) );

		// Draw skeletons
		uint32_t i = 0;
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.begin(); skeletonIt != mSkeletons.end(); ++skeletonIt, i++ ) {
			gl::color( mKinect->getUserColor( i ) );
			for ( Skeleton::const_iterator boneIt = skeletonIt->begin(); boneIt != skeletonIt->end(); ++boneIt ) {
				const Bone& bone		= boneIt->second;
				Vec3f position			= bone.getPosition();
				Vec3f destination		= skeletonIt->at( bone.getStartJoint() ).getPosition();
				Vec2f positionScreen	= Vec2f( mKinect->getSkeletonColorPos( position ) );
				Vec2f destinationSceen	= Vec2f( mKinect->getSkeletonColorPos( destination ) );
				gl::drawLine( positionScreen, destinationSceen );
				gl::drawSolidCircle( positionScreen, 10.0f, 16 );
			}
		}

		gl::popMatrices();
	}
}

void SkeletonBitmapApp::keyDown( KeyEvent event )
{
	switch ( event.getCode() ) {
	case KeyEvent::KEY_q:
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

void SkeletonBitmapApp::onFrame( Frame frame, const DeviceOptions &deviceOptions )
{
	mSkeletons = frame.getSkeletons();
	if ( mTexture ) {
		mTexture->update( frame.getColorSurface() );
	} else {
		mTexture = gl::Texture::create( frame.getColorSurface() );
	}
}

void SkeletonBitmapApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
}

void SkeletonBitmapApp::screenShot()
{
	writeImage( getAppPath() / ( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void SkeletonBitmapApp::setup()
{
	glLineWidth( 2.0f );
	gl::color( ColorAf::white() );

	mKinect = Kinect::create();
	mKinect->connectEventHandler( &SkeletonBitmapApp::onFrame, this );
	mKinect->start( DeviceOptions().enableDepth( false ) );
}

void SkeletonBitmapApp::shutdown()
{
	mKinect->stop();
}

void SkeletonBitmapApp::update()
{
	if ( mKinect->isCapturing() ) {
		mKinect->update();
	} else {
		if ( getElapsedFrames() % 90 == 0 ) {
			mKinect->start();
		}
	}
}

CINDER_APP_BASIC( SkeletonBitmapApp, RendererGl )

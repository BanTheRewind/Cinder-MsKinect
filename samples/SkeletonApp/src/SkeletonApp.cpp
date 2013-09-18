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
#include "cinder/Camera.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"
#include "cinder/Utilities.h"
#include "Kinect.h"

/* 
* This application demonstrates how to acquire and display 
* skeleton data.
*/ 
class SkeletonApp : public ci::app::AppBasic 
{
public:
	void	draw();
	void	keyDown( ci::app::KeyEvent event );
	void	prepareSettings( ci::app::AppBasic::Settings* settings );
	void	setup();
	void	shutdown();
	void	update();

private:
	MsKinect::DeviceRef				mDevice;
	std::vector<MsKinect::Skeleton>	mSkeletons;
	void							onFrame( MsKinect::Frame frame, const MsKinect::DeviceOptions &deviceOptions );

	ci::CameraPersp					mCamera;

	void							screenShot();
};

using namespace ci;
using namespace ci::app;
using namespace MsKinect;
using namespace std;

void SkeletonApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::gray( 0.1f ) );

	if ( mDevice->isCapturing() ) {
		gl::setMatrices( mCamera );

		uint32_t i = 0;
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.begin(); skeletonIt != mSkeletons.end(); ++skeletonIt, ++i ) {

			Colorf color = mDevice->getUserColor( i );

			for ( Skeleton::const_iterator boneIt = skeletonIt->begin(); boneIt != skeletonIt->end(); ++boneIt ) {
				gl::color( color );

				const Bone& bone	= boneIt->second;
				Vec3f position		= bone.getPosition();
				Matrix44f transform	= bone.getAbsoluteRotationMatrix();
				Vec3f direction		= transform.transformPoint( position ).normalized();
				direction			*= 0.05f;
				position.z			*= -1.0f;

				glLineWidth( 2.0f );
				JointName startJoint = bone.getStartJoint();
				if ( skeletonIt->find( startJoint ) != skeletonIt->end() ) {
					Vec3f destination	= skeletonIt->find( startJoint )->second.getPosition();
					destination.z		*= -1.0f;
					gl::drawLine( position, destination );
				}

				gl::drawSphere( position, 0.025f, 16 );

				glLineWidth( 0.5f );
				gl::color( ColorAf::white() );
				gl::drawVector( position, position + direction, 0.05f, 0.01f );
			}
		}
	}
}

void SkeletonApp::keyDown( KeyEvent event )
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

void SkeletonApp::onFrame( Frame frame, const DeviceOptions &deviceOptions )
{
	mSkeletons = frame.getSkeletons();
}

void SkeletonApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
}

void SkeletonApp::screenShot()
{
	writeImage( getAppPath() / ( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void SkeletonApp::setup()
{
	mDevice = Device::create();
	mDevice->connectEventHandler( &SkeletonApp::onFrame, this );
	mDevice->start( DeviceOptions().enableDepth( false ).enableColor( false ) );
	mDevice->removeBackground();
	mDevice->setTransform( Device::TRANSFORM_SMOOTH );
	
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 2.0f ), Vec3f::zero() );
	mCamera.setPerspective( 45.0f, getWindowAspectRatio(), 0.01f, 1000.0f );
}

void SkeletonApp::shutdown()
{
	mDevice->stop();
}

void SkeletonApp::update()
{
	if ( !mDevice->isCapturing() ) {
		if ( getElapsedFrames() % 90 == 0 ) {
			mDevice->start();
		}
	}
}

CINDER_APP_BASIC( SkeletonApp, RendererGl )

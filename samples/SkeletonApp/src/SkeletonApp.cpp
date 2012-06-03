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
#include "Kinect.h"

/* 
* This application demonstrates how to acquire and display 
* skeleton data.
*/ 
class SkeletonApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void keyDown( ci::app::KeyEvent event );
	void prepareSettings( ci::app::AppBasic::Settings * settings );
	void setup();
	void shutdown();
	void update();

private:

	// Kinect
	void								drawSegment( const KinectSdk::Skeleton & skeleton, 
													 const std::vector<KinectSdk::JointName> & joints );
	KinectSdk::KinectRef				mKinect;
	std::vector<KinectSdk::Skeleton>	mSkeletons;

	// Skeleton segments
	void												defineBody();
	std::vector<KinectSdk::JointName>					mBody;
	std::vector<KinectSdk::JointName>					mLeftArm;
	std::vector<KinectSdk::JointName>					mLeftLeg;
	std::vector<KinectSdk::JointName>					mRightArm;
	std::vector<KinectSdk::JointName>					mRightLeg;
	std::vector<std::vector<KinectSdk::JointName> >		mSegments;

	// Camera
	ci::CameraPersp										mCamera;

	// Save screenshot
	void												screenShot();

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// Define body drawing
void SkeletonApp::defineBody()
{

	// Bail if segments not defined
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
	mRightLeg.push_back( NUI_SKELETON_POSITION_HIP_CENTER );
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
void SkeletonApp::draw()
{

	// Clear window
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::gray( 0.1f ) );

	// We're capturing
	if ( mKinect->isCapturing() ) {

		// Set up camera for 3D
		gl::setMatrices( mCamera );

		// Iterate through skeletons
		uint32_t i = 0;
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.cbegin(); skeletonIt != mSkeletons.cend(); ++skeletonIt, i++ ) {

			// Valid skeletons have all joints
			if ( skeletonIt->size() == JointName::NUI_SKELETON_POSITION_COUNT ) {

				// Set color
				Colorf color = mKinect->getUserColor( i );

				// Draw body
				glLineWidth( 2.0f );
				gl::color( color );
				for ( vector<vector<JointName> >::const_iterator segmentIt = mSegments.cbegin(); segmentIt != mSegments.cend(); ++segmentIt ) {
					drawSegment( * skeletonIt, * segmentIt );
				}

				// Iterate through joints
				glLineWidth( 0.5f );
				for ( Skeleton::const_iterator jointIt = skeletonIt->cbegin(); jointIt != skeletonIt->cend(); ++jointIt ) {
					
					// Get position and rotation
					Vec3f position = jointIt->second.getPosition();
					Matrix44f transform = jointIt->second.getAbsoluteRotationMatrix();
					Vec3f direction = transform.transformPoint( position ).normalized();
					direction *= 0.05f;

					// Draw joint
					gl::color( color );
					gl::drawSphere( position, 0.025f, 16 );
					
					// Draw joint orientation
					gl::color( ColorAf::white() );
					gl::drawVector( position, position + direction, 0.05f, 0.01f );

				}

			}

		}

	}

}

// Draw segment
void SkeletonApp::drawSegment( const Skeleton &skeleton, const vector<JointName> &joints )
{
	glBegin( GL_LINES );
	for ( uint32_t i = 0; i < joints.size() - 1; i++ ) {
		gl::vertex( skeleton.at( joints.at( i ) ).getPosition() );
		gl::vertex( skeleton.at( joints.at( i + 1 ) ).getPosition() );
	}
	glEnd();
}

// Handles key press
void SkeletonApp::keyDown( KeyEvent event )
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
void SkeletonApp::prepareSettings( Settings * settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
}

// Take screen shot
void SkeletonApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void SkeletonApp::setup()
{

	// Start Kinect
	mKinect = Kinect::create();
	mKinect->start( DeviceOptions().enableDepth( false ).enableVideo( false ) );
	mKinect->removeBackground();

	// Set the skeleton smoothing to remove jitters. Better smoothing means
	// less jitters, but a slower response time.
	mKinect->setTransform( Kinect::TRANSFORM_SMOOTH );

	// Set up camera
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 2.0f ), Vec3f::zero() );
	mCamera.setPerspective( 45.0f, getWindowAspectRatio(), 0.01f, 1000.0f );

	// Define drawing body
	defineBody();

}

// Called on exit
void SkeletonApp::shutdown()
{

	// Stop input
	mKinect->stop();

	// Clean up
	mBody.clear();
	mLeftArm.clear();
	mLeftLeg.clear();
	mRightArm.clear();
	mRightLeg.clear();
	mSegments.clear();
	mSkeletons.clear();

}

// Runs update logic
void SkeletonApp::update()
{

	// Kinect is capturing
	if ( mKinect->isCapturing() ) {
	
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
CINDER_APP_BASIC( SkeletonApp, RendererGl )

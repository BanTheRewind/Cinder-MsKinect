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
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "Kinect.h"
#include "Resources.h"

/*
* This application demonstrates how to combine a vertex 
* buffer object with the Kinect's depth image to create
* a point cloud on the GPU. It also uses skeleton data
* to track the user with the camera.
*/
class PointCloudGpuApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void								draw();
	void								keyDown( ci::app::KeyEvent event );
	void								prepareSettings( ci::app::AppBasic::Settings *settings );
	void								resize( ci::app::ResizeEvent event );
	void								shutdown();
	void								setup();
	void								update();

private:

	// Kinect
	int32_t								mCallbackDepthId;
	int32_t								mCallbackSkeletonId;
	KinectSdk::KinectRef				mKinect;
	std::vector<KinectSdk::Skeleton>	mSkeletons;
	ci::gl::Texture						mTextureDepth;
	void								onDepthData( ci::Surface16u  surface, const KinectSdk::DeviceOptions &deviceOptions );
	void								onSkeletonData( std::vector<KinectSdk::Skeleton> skeletons, const KinectSdk::DeviceOptions &deviceOptions );

	// VBO
	ci::gl::GlslProg					mShader;
	ci::gl::VboMesh						mVboMesh;

	// Camera
	ci::CameraPersp						mCamera;
	ci::Vec3f							mEyePoint;
	ci::Vec3f							mLookAt;
	ci::Vec3f							mRotation;

	// Background image
	ci::gl::Texture						mBackground;

	// Parameters
	float								mDepth;
	float								mFrameRate;
	bool								mFullScreen;
	bool								mFullScreenPrev;
	ci::params::InterfaceGl				mParams;
	float								mPointSize;
	bool								mRemoveBackground;
	bool								mRemoveBackgroundPrev;
	ci::Vec3f							mScale;
	
	// Save screen shot
	void								screenShot();

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// Render
void PointCloudGpuApp::draw()
{

	// Set up window
	gl::clear( ColorAf::black(), true );
	gl::setMatrices( mCamera );
	gl::color( ColorAf::white() );

	// Draw world
	if ( mBackground ) {
		mBackground.bind( 0 );
		gl::drawSphere( Vec3f::zero(), 3000.0f, 32 );
		mBackground.unbind();
	}

	// Check texture and VBO
	if ( mTextureDepth && mVboMesh ) {

		// Position world
		gl::pushMatrices();
		gl::scale( -1.0f, -1.0f, -1.0f );
		gl::rotate( mRotation );

		// Bind texture
		mTextureDepth.bind( 0 );
		
		// Bind and configure shader
		mShader.bind();
		mShader.uniform( "depth", mDepth );
		mShader.uniform( "scale", mScale );
		mShader.uniform( "tex0", 0 );

		// Draw VBO
		glPointSize( mPointSize );
		gl::draw( mVboMesh );

		// Stop drawing
		mShader.unbind();
		mTextureDepth.unbind();
		gl::popMatrices();

	}

	// Draw params
	params::InterfaceGl::draw();

}

// Handles key press
void PointCloudGpuApp::keyDown( KeyEvent event )
{

	// Key on key...
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

void PointCloudGpuApp::onDepthData( Surface16u surface, const DeviceOptions &deviceOptions )
{
	mTextureDepth = gl::Texture(  surface );
}

void PointCloudGpuApp::onSkeletonData( vector<Skeleton> skeletons, const DeviceOptions &deviceOptions )
{
	mSkeletons = skeletons;
}

// Prepare window
void PointCloudGpuApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
}

// Handles window resizw
void PointCloudGpuApp::resize(ResizeEvent event)
{

	// Set up OpenGL
	gl::enable( GL_DEPTH_TEST );
	gl::enable( GL_LINE_SMOOTH );
	glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
	gl::enable( GL_POINT_SMOOTH );
	glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
	gl::enable( GL_POLYGON_SMOOTH );
	glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
	gl::enable( GL_TEXTURE_2D );
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableAlphaBlending();
	
	// Reset camera
	mEyePoint = Vec3f( 0.0f, 0.0f, 100.0f );
	mLookAt = Vec3f::zero();
	mRotation = Vec3f::zero();
	mCamera.lookAt( mEyePoint, mLookAt );
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 1.0f, 6000.0f );

}

// Take screen shot
void PointCloudGpuApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void PointCloudGpuApp::setup()
{

	// Start Kinect
	mKinect = Kinect::create();
	mKinect->removeBackground();
	mKinect->enableUserColor( false );
	mKinect->start( DeviceOptions().enableVideo( false ) );

	// Add callbacks
	mCallbackDepthId = mKinect->addDepthCallback( &PointCloudGpuApp::onDepthData, this );
	mCallbackSkeletonId = mKinect->addSkeletonTrackingCallback( &PointCloudGpuApp::onSkeletonData, this );

	// VBO data
	vector<uint32_t> vboIndices;
	gl::VboMesh::Layout vboLayout;
	vector<Vec3f> vboPositions;
	vector<Vec2f> vboTexCoords;

	// Set up VBO layout
	vboLayout.setStaticIndices();
	vboLayout.setStaticPositions();
	vboLayout.setStaticTexCoords2d();
	
	// VBO dimensions
	int32_t height = 240;
	int32_t width = 320;

	// Define VBO data
	for ( int32_t x = 0; x < width; x++) {
		for ( int32_t y = 0; y < height; y++ ) {
			vboIndices.push_back( (uint32_t)( x * height + y ) );
			vboTexCoords.push_back( Vec2f( (float)x / (float)( width - 1 ), (float)y / (float)( height - 1 ) ) );
			vboPositions.push_back( Vec3f(
				( vboTexCoords.rbegin()->x * 2.0f - 1.0f ) * (float)width, 
				( vboTexCoords.rbegin()->y * 2.0f - 1.0f ) * (float)height, 
				0.0f ) );
		}
	}

	// Build VBO
	mVboMesh = gl::VboMesh( vboPositions.size(), vboIndices.size(), vboLayout, GL_POINTS );
	mVboMesh.bufferIndices( vboIndices );
	mVboMesh.bufferPositions( vboPositions );
	mVboMesh.bufferTexCoords2d( 0, vboTexCoords );
	mVboMesh.unbindBuffers();

	// Load background
	mBackground = gl::Texture( loadImage( loadResource( RES_IMAGE_BACKGROUND ) ) );
	mBackground.setWrap( GL_REPEAT, GL_REPEAT );
	mBackground.setMinFilter( GL_LINEAR );
	mBackground.setMagFilter( GL_LINEAR );

	// Load shader
	mShader = gl::GlslProg( loadResource( RES_SHADER_USER_VERT ), loadResource( RES_SHADER_USER_FRAG ) );

	// Clean up
	vboIndices.clear();
	vboPositions.clear();
	vboTexCoords.clear();

	// Run first window resize
	resize( ResizeEvent( getWindowSize() ) );

	// Set default properties
	mDepth = 500.0f;
	mFrameRate = 0.0f;
	mFullScreen = false;
	mFullScreenPrev = mFullScreen;
	mPointSize = 2.0f;
	mRemoveBackground = true;
	mRemoveBackgroundPrev = mRemoveBackground;
	mScale = Vec3f( 1.0f, 1.0f, 5.0f );

	// Setup the parameters
	mParams = params::InterfaceGl( "Parameters", Vec2i( 200, 500 ) );
	mParams.addSeparator();
	mParams.addText( "APPLICATION" );
	mParams.addParam( "Frame rate",			&mFrameRate,									"", true												);
	mParams.addParam( "Full screen",		&mFullScreen,									"key=f"													);
	mParams.addButton( "Screen shot",		bind( &PointCloudGpuApp::screenShot, this ),	"key=s"													);
	mParams.addButton( "Quit",				bind( &PointCloudGpuApp::quit, this ),			"key=esc"												);
	mParams.addSeparator();
	mParams.addText( "CAMERA" );
	mParams.addParam( "Eye point",			&mEyePoint																								);
	mParams.addParam( "Look at",			&mLookAt																								);
	mParams.addParam( "Rotation",			&mRotation																								);
	mParams.addSeparator();
	mParams.addText( "USER" );
	mParams.addParam( "Depth",				&mDepth,										"min=0.0 max=2000.0 step=1.0 keyIncr=Z keyDecr=z"		);
	mParams.addParam( "Point size",			&mPointSize,									"min=0.025 max=100.000 step=0.001 keyIncr=P keyDecr=p"	);
	mParams.addParam( "Remove background",	&mRemoveBackground,								"key=b"													);
	mParams.addParam( "Scale",				&mScale																									);

}

// Quit
void PointCloudGpuApp::shutdown()
{

	// Stop Kinect input
	mKinect->removeCallback( mCallbackDepthId );
	mKinect->removeCallback( mCallbackSkeletonId );
	mKinect->stop();

}

// Runs update logic
void PointCloudGpuApp::update()
{

	// Update frame rate
	mFrameRate = getAverageFps();

	// Toggle fullscreen
	if ( mFullScreen != mFullScreenPrev ) {
		setFullScreen( mFullScreen );
		mFullScreenPrev = mFullScreen;
	}

	// Toggle background
	if (mRemoveBackground != mRemoveBackgroundPrev)
	{
		mKinect->removeBackground(mRemoveBackground);
		mRemoveBackgroundPrev = mRemoveBackground;
	}

	// Update Kinect
	if ( mKinect->isCapturing() ) {
		mKinect->update();

		// Find first active skeleton
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.cbegin(); skeletonIt != mSkeletons.cend(); ++skeletonIt ) {
					
			// Valid skeletons have all joints
			if ( skeletonIt->size() == (uint32_t)JointName::NUI_SKELETON_POSITION_COUNT ) {

				// Look at spine
				Vec3f spine = skeletonIt->at( JointName::NUI_SKELETON_POSITION_SPINE ).getPosition() * -mEyePoint.z;
				mLookAt.x = spine.x;
				mLookAt.y = spine.y;
				mEyePoint.x = -mLookAt.x * 0.25f;
				mEyePoint.y = -mLookAt.y * 0.125f;

			}

		}

	} else {

		// If Kinect initialization failed, try again every 90 frames
		if ( getElapsedFrames() % 90 == 0 ) {
			mKinect->start();
		}

	}

	// Update camera
	mCamera.lookAt( mEyePoint, mLookAt );

}

// Run application
CINDER_APP_BASIC( PointCloudGpuApp, RendererGl )

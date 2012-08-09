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
class MeshApp : public ci::app::AppBasic 
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
	float								mBrightTolerance;
	KinectSdk::KinectRef				mKinect;
	float								mMeshUvMix;
	bool								mRemoveBackground;
	bool								mRemoveBackgroundPrev;
	ci::Vec3f							mScale;
	bool								mShowVideo;
	std::vector<KinectSdk::Skeleton>	mSkeletons;
	ci::gl::Texture						mTextureDepth;
	ci::gl::Texture::Format				mTextureFormat;
	ci::gl::Texture						mTextureVideo;
	float								mVideoOffsetX;
	float								mVideoOffsetY;

	// Kinect callbacks
	int32_t								mCallbackDepthId;
	int32_t								mCallbackSkeletonId;
	int32_t								mCallbackVideoId;
	void								onDepthData( ci::Surface16u surface, const KinectSdk::DeviceOptions &deviceOptions );
	void								onSkeletonData( std::vector<KinectSdk::Skeleton> skeletons, const KinectSdk::DeviceOptions &deviceOptions );
	void								onVideoData( ci::Surface8u surface, const KinectSdk::DeviceOptions &deviceOptions );

	// VBO
	void								initMesh();
	ci::gl::VboMesh						mVboMesh;
	ci::gl::GlslProg					mShader;

	// Camera
	ci::CameraPersp						mCamera;
	ci::Vec3f							mEyePoint;
	ci::Vec3f							mLookAt;
	ci::Vec3f							mRotation;

	// Lighting
	ci::ColorAf							mLightAmbient;
	ci::ColorAf							mLightDiffuse;
	ci::Vec3f							mLightPosition;
	float								mLightShininess;
	ci::ColorAf							mLightSpecular;

	// Window
	float								mFrameRate;
	bool								mFullScreen;
	bool								mFullScreenPrev;

	// Parameters
	ci::params::InterfaceGl				mParams;

	// Debugging
	void								screenShot();
	void								trace( const std::string & message );

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

// VBO dimensions
const int32_t	kMeshHeight	= 240;
const int32_t	kMeshWidth	= 320;

// Render
void MeshApp::draw()
{

	// Set up window
	gl::clear( ColorAf::black(), true );
	gl::setMatrices( mCamera );
	gl::color( ColorAf::white() );

	// Check texture and VBO
	if ( mTextureDepth && mTextureVideo && mVboMesh ) {

		// Position world
		gl::pushMatrices();
		gl::scale( -1.0f, -1.0f, -1.0f );
		gl::rotate( mRotation );

		// Bind textures
		mTextureDepth.bind( 0 );
		mTextureVideo.bind( 1 );
		
		// Bind and configure shader
		mShader.bind();
		mShader.uniform( "brightTolerance",	mBrightTolerance 						);
		mShader.uniform( "eyePoint",		mEyePoint 								);
		mShader.uniform( "lightAmbient",	mLightAmbient 							);
		mShader.uniform( "lightDiffuse",	mLightDiffuse 							);
		mShader.uniform( "lightPosition",	mLightPosition 							);
		mShader.uniform( "lightSpecular",	mLightSpecular 							);
		mShader.uniform( "positions",		0 										);
		mShader.uniform( "scale",			mScale 									);
		mShader.uniform( "showVideo",		mShowVideo 								);
		mShader.uniform( "shininess",		mLightShininess 						);
		mShader.uniform( "video",			1 										);
		mShader.uniform( "videoOffset",		Vec2f( mVideoOffsetX, mVideoOffsetY )	);
		mShader.uniform( "uvmix",			mMeshUvMix								);

		// Draw VBO
		gl::draw( mVboMesh );

		// Stop drawing
		mShader.unbind();
		mTextureDepth.unbind();
		mTextureVideo.unbind();
		gl::popMatrices();

	}

	// Draw params
	params::InterfaceGl::draw();

}

// Initialize mesh
void MeshApp::initMesh()
{

	// VBO dimensions as floats
	float heightf	= (float)kMeshHeight;
	float widthf	= (float)kMeshWidth;

	// VBO data
	gl::VboMesh::Layout vboLayout;
	vector<uint32_t> vboIndices;
	vector<Vec3f> vboPositions;
	vector<Vec2f> vboTexCoords;

	// Set up VBO layout
	vboLayout.setStaticIndices();
	vboLayout.setStaticPositions();
	vboLayout.setStaticTexCoords2d();

	// Define corners of quad (two triangles)
	vector<Vec2f> quad;
	quad.push_back( Vec2f( 0.0f, 0.0f ) );
	quad.push_back( Vec2f( 0.0f, 1.0f ) );
	quad.push_back( Vec2f( 1.0f, 0.0f ) );
	quad.push_back( Vec2f( 1.0f, 0.0f ) );
	quad.push_back( Vec2f( 0.0f, 1.0f ) );
	quad.push_back( Vec2f( 1.0f, 1.0f ) );

	// Iterate through rows in the mesh
	for ( int32_t y = 0; y < kMeshHeight; y++ ) {

		// Iterate through the vectors in this row
		for ( int32_t x = 0; x < kMeshWidth; x++ ) {

			// Get vertices as floats
			float xf = (float)x;
			float yf = (float)y;

			// Add position in world coordinates
			vboPositions.push_back( Vec3f( xf - widthf * 0.5f, yf - heightf * 0.5f, 0.0f ) );

			// Use percentage of the position for the texture coordinate
			vboTexCoords.push_back( Vec2f( xf / widthf, yf / heightf ) );

			// Do not add a quad to the bottom or right side of the mesh
			if ( x < kMeshWidth && y < kMeshHeight ) {

				// Iterate through points in the quad to set indices
				for ( vector<Vec2f>::const_iterator vertIt = quad.cbegin(); vertIt != quad.cend(); ++vertIt ) {

					// Get vertices as floats
					xf = (float)x + vertIt->x;
					yf = (float)y + vertIt->y;

					// Set the index of the vertex in the VBO so it is
					// numbered left to right, top to bottom
					vboIndices.push_back( (uint32_t)( xf + yf * widthf ) );

				}

			}

		}

	}

	// Build VBO
	mVboMesh = gl::VboMesh( vboPositions.size(), vboIndices.size(), vboLayout, GL_TRIANGLES );
	mVboMesh.bufferIndices( vboIndices );
	mVboMesh.bufferPositions( vboPositions );
	mVboMesh.bufferTexCoords2d( 0, vboTexCoords );
	mVboMesh.unbindBuffers();

	// Load and compile shader
	try {
		mShader = gl::GlslProg( loadResource( RES_SHADER_USER_VERT ), loadResource( RES_SHADER_USER_FRAG ) );
	} catch ( gl::GlslProgCompileExc ex ) {
		trace( "Unable to compile shader: " );
		trace( ex.what() );
		quit();
	}

	// Clean up
	vboIndices.clear();
	vboPositions.clear();
	vboTexCoords.clear();

}

// Handles key press
void MeshApp::keyDown( KeyEvent event )
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

// Receives depth data 
void MeshApp::onDepthData( Surface16u surface, const DeviceOptions &deviceOptions )
{
	mTextureDepth = gl::Texture( surface );
}

// Receives skeleton data
void MeshApp::onSkeletonData( vector<Skeleton> skeletons, const DeviceOptions &deviceOptions )
{
	mSkeletons = skeletons;
}

// Receives video data
void MeshApp::onVideoData( Surface8u surface, const DeviceOptions &deviceOptions )
{
	if ( mTextureVideo ) {
		mTextureVideo.update( surface, surface.getBounds() );
	} else {
		mTextureVideo = gl::Texture( surface );
		mTextureVideo.setWrap( GL_REPEAT, GL_REPEAT );
	}
}

// Prepare window
void MeshApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate( 60.0f );
}

// Handles window resize
void MeshApp::resize( ResizeEvent event )
{

	// Reset camera
	mEyePoint = Vec3f( 0.0f, 0.0f, 100.0f );
	mLookAt = Vec3f::zero();
	mRotation = Vec3f::zero();
	mCamera.lookAt( mEyePoint, mLookAt );
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 0.1f, 15000.0f );

	// Set up OpenGL
	gl::enable( GL_DEPTH_TEST );
	glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
	gl::enable( GL_POLYGON_SMOOTH );
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableAlphaBlending();
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	gl::color( ColorAf::white() );

}

// Take screen shot
void MeshApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void MeshApp::setup()
{

	// Start Kinect
	mKinect = Kinect::create();
	mKinect->enableUserColor( false );
	mKinect->removeBackground();
	mKinect->start();

	// Add callbacks
	mCallbackDepthId	= mKinect->addDepthCallback( &MeshApp::onDepthData, this );
	mCallbackSkeletonId	= mKinect->addSkeletonTrackingCallback( &MeshApp::onSkeletonData, this );
	mCallbackVideoId	= mKinect->addVideoCallback( &MeshApp::onVideoData, this );

	// Set up the light. This application does not actually use OpenGL 
	// lighting. Instead, it passes a light position and color 
	// values to the shader. Per fragment lighting is calculated in GLSL.
	mLightAmbient	= ColorAf( 0.0f, 0.0f, 0.0f, 1.0f );
	mLightDiffuse	= ColorAf( 0.5f, 0.5f, 0.5f, 1.0f );
	mLightPosition	= Vec3f( 0.0f, -600.0f, 180.0f );
	mLightShininess	= 2.0f;
	mLightSpecular	= ColorAf( 1.0f, 1.0f, 1.0f, 1.0f );

	// Set default properties
	mBrightTolerance		= 0.2f;
	mFrameRate				= 0.0f;
	mFullScreen				= isFullScreen();
	mFullScreenPrev			= mFullScreen;
	mMeshUvMix				= 0.2f;
	mRemoveBackground		= true;
	mRemoveBackgroundPrev	= mRemoveBackground;
	mScale					= Vec3f( 1.0f, 1.0f, 500.0f );
	mShowVideo				= false;
	mVideoOffsetX			= 0.0f;
	mVideoOffsetY			= 0.0f;

	// Create the parameters bar
	mParams = params::InterfaceGl( "Parameters", Vec2i( 250, 500 ) );
	mParams.addSeparator( "" );
	mParams.addParam( "Bright tolerance",	&mBrightTolerance,					"min=0.000 max=1.000 step=0.001 keyDecr=b keyIncr=B"		);
	mParams.addParam( "Remove background",	&mRemoveBackground,					"key=c"														);
	mParams.addParam( "Scale",				&mScale																							);
	mParams.addSeparator();
	mParams.addParam( "Eye point",			&mEyePoint																						);
	mParams.addParam( "Look at",			&mLookAt																						);
	mParams.addParam( "Rotation",			&mRotation																						);
	mParams.addSeparator();
	mParams.addParam( "Show video",			&mShowVideo,						"key=d"														);
	mParams.addParam( "Video offset X",		&mVideoOffsetX,						"min=0.000 max=1.000 step=0.001 keyDecr=e keyIncr=E"		);
	mParams.addParam( "Video offset Y",		&mVideoOffsetY,						"min=0.000 max=1.000 step=0.001 keyDecr=f keyIncr=F"		);
	mParams.addSeparator();
	mParams.addParam( "Light position",		&mLightPosition																					);
	mParams.addParam( "Light shininess",	&mLightShininess,					"min=0.000 max=10000.000 step=0.001 keyDecr=g keyIncr=G"	);
	mParams.addSeparator();
	mParams.addParam( "Frame rate",			&mFrameRate,						"", true													);
	mParams.addParam( "Full screen",		&mFullScreen,						"key=h"														);
	mParams.addButton( "Save screen shot",	bind( &MeshApp::screenShot, this ),	"key=space"													);
	mParams.addButton( "Quit",				bind( &MeshApp::quit, this ),		"key=esc"													);

	// Initialize texture
	mTextureFormat.setInternalFormat( GL_RGBA_FLOAT32_ATI );
	mTextureDepth = gl::Texture( Surface32f( kMeshWidth, kMeshHeight, false, SurfaceChannelOrder::RGBA ), mTextureFormat );

	// Initialize mesh
	initMesh();

	// Run first window resize
	resize( ResizeEvent( getWindowSize() ) );

}

// Quit
void MeshApp::shutdown()
{

	// Stop input
	mKinect->removeCallback( mCallbackDepthId );
	mKinect->removeCallback( mCallbackSkeletonId );
	mKinect->removeCallback( mCallbackVideoId );
	mKinect->stop();

	// Clean up
	mSkeletons.clear();

}

// Debug trace
void MeshApp::trace( const string & message )
{

	// Write to console and debug window
	console() << message << "\n";
	OutputDebugStringA( ( message + "\n" ).c_str() );

}

// Runs update logic
void MeshApp::update()
{

	// Update frame rate
	mFrameRate = getAverageFps();

	// Toggle fullscreen
	if ( mFullScreen != mFullScreenPrev ) {
		setFullScreen( mFullScreen );
		mFullScreenPrev = mFullScreen;
	}

	// Toggle background
	if ( mRemoveBackground != mRemoveBackgroundPrev ) {
		mKinect->removeBackground( mRemoveBackground );
		mRemoveBackgroundPrev = mRemoveBackground;
	}

	// Kinect is running
	if ( mKinect->isCapturing() ) {
		mKinect->update();

		// Find first active skeleton...
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.cbegin(); skeletonIt != mSkeletons.cend(); ++skeletonIt ) {

			// Valid skeletons have all joints available
			if ( skeletonIt->size() == (uint32_t)JointName::NUI_SKELETON_POSITION_COUNT ) {

				// Subtle camera follow
				Vec3f spine = skeletonIt->at( JointName::NUI_SKELETON_POSITION_SPINE ).getPosition() * mEyePoint.z;
				mLookAt.x = spine.x * 0.05f;
				mLookAt.y = spine.y * 0.05f;
				mEyePoint.x = -mLookAt.x * 0.5f;
				mEyePoint.y = -mLookAt.y * 0.25f;

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
CINDER_APP_BASIC( MeshApp, RendererGl )

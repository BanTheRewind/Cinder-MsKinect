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
	void								draw();
	void								prepareSettings( ci::app::AppBasic::Settings* settings );
	void								resize();
	void								shutdown();
	void								setup();
	void								update();
private:
	MsKinect::DeviceRef					mDevice;
	std::vector<MsKinect::Skeleton>		mSkeletons;
	ci::gl::TextureRef					mTextureDepth;
	void								onFrame( MsKinect::Frame frame, const MsKinect::DeviceOptions& deviceOptions );

	ci::gl::GlslProgRef					mShader;
	ci::gl::VboMeshRef					mVboMesh;

	ci::CameraPersp						mCamera;
	ci::Vec3f							mEyePoint;
	ci::Vec3f							mLookAt;
	ci::Vec3f							mRotation;

	ci::gl::TextureRef					mBackground;

	float								mDepth;
	float								mFrameRate;
	bool								mFullScreen;
	bool								mFullScreenPrev;
	ci::params::InterfaceGlRef			mParams;
	float								mPointSize;
	bool								mRemoveBackground;
	bool								mRemoveBackgroundPrev;
	ci::Vec3f							mScale;
	
	void								screenShot();
};

using namespace ci;
using namespace ci::app;
using namespace MsKinect;
using namespace std;

void PointCloudGpuApp::draw()
{
	gl::clear( ColorAf::black(), true );
	gl::setMatrices( mCamera );
	gl::color( ColorAf::white() );

	if ( mBackground ) {
		mBackground->bind( 0 );
		gl::drawSphere( Vec3f::zero(), 3000.0f, 32 );
		mBackground->unbind();
	}

	if ( mTextureDepth && mVboMesh ) {

		gl::pushMatrices();
		gl::scale( -1.0f, -1.0f, -1.0f );
		gl::rotate( mRotation );

		mTextureDepth->bind( 0 );
		
		mShader->bind();
		mShader->uniform( "depth", mDepth );
		mShader->uniform( "scale", mScale );
		mShader->uniform( "tex0", 0 );

		glPointSize( mPointSize );
		gl::draw( mVboMesh );

		mShader->unbind();
		mTextureDepth->unbind();
		gl::popMatrices();

	}

	mParams->draw();
}

void PointCloudGpuApp::onFrame( Frame frame, const DeviceOptions& deviceOptions )
{
	if ( mTextureDepth ) {
		mTextureDepth->update( Surface32f( frame.getDepthSurface() ) );
	} else {
		mTextureDepth	= gl::Texture::create( frame.getDepthSurface() );
	}
	mSkeletons			= frame.getSkeletons();
}

void PointCloudGpuApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
}

void PointCloudGpuApp::resize()
{
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
	
	mEyePoint = Vec3f( 0.0f, 0.0f, 100.0f );
	mLookAt = Vec3f::zero();
	mRotation = Vec3f::zero();
	mCamera.lookAt( mEyePoint, mLookAt );
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 1.0f, 6000.0f );
}

void PointCloudGpuApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void PointCloudGpuApp::setup()
{
	mDevice = Device::create();
	mDevice->connectEventHandler( &PointCloudGpuApp::onFrame, this );
	mDevice->removeBackground();
	mDevice->enableUserColor( false );
	mDevice->start( DeviceOptions().enableColor( false ) );

	vector<uint32_t> vboIndices;
	gl::VboMesh::Layout vboLayout;
	vector<Vec3f> vboPositions;
	vector<Vec2f> vboTexCoords;

	vboLayout.setStaticIndices();
	vboLayout.setStaticPositions();
	vboLayout.setStaticTexCoords2d();
	
	int32_t height = 240;
	int32_t width = 320;

	for ( int32_t x = 0; x < width; ++x ) {
		for ( int32_t y = 0; y < height; ++y ) {
			vboIndices.push_back( (uint32_t)( x * height + y ) );
			vboTexCoords.push_back( Vec2f( (float)x / (float)( width - 1 ), (float)y / (float)( height - 1 ) ) );
			vboPositions.push_back( Vec3f(
				( vboTexCoords.rbegin()->x * 2.0f - 1.0f ) * (float)width, 
				( vboTexCoords.rbegin()->y * 2.0f - 1.0f ) * (float)height, 
				0.0f ) );
		}
	}

	mVboMesh = gl::VboMesh::create( vboPositions.size(), vboIndices.size(), vboLayout, GL_POINTS );
	mVboMesh->bufferIndices( vboIndices );
	mVboMesh->bufferPositions( vboPositions );
	mVboMesh->bufferTexCoords2d( 0, vboTexCoords );
	mVboMesh->unbindBuffers();

	mBackground = gl::Texture::create( loadImage( loadResource( RES_IMAGE_BACKGROUND ) ) );
	mBackground->setWrap( GL_REPEAT, GL_REPEAT );
	mBackground->setMinFilter( GL_LINEAR );
	mBackground->setMagFilter( GL_LINEAR );

	mShader = gl::GlslProg::create( loadResource( RES_SHADER_USER_VERT ), loadResource( RES_SHADER_USER_FRAG ) );

	vboIndices.clear();
	vboPositions.clear();
	vboTexCoords.clear();

	resize();

	mDepth = 500.0f;
	mFrameRate = 0.0f;
	mFullScreen = false;
	mFullScreenPrev = mFullScreen;
	mPointSize = 2.0f;
	mRemoveBackground = true;
	mRemoveBackgroundPrev = mRemoveBackground;
	mScale = Vec3f( 1.0f, 1.0f, 5.0f );

	mParams = params::InterfaceGl::create( "Parameters", Vec2i( 200, 500 ) );
	mParams->addSeparator();
	mParams->addText( "APPLICATION" );
	mParams->addParam( "Frame rate",		&mFrameRate,									"", true												);
	mParams->addParam( "Full screen",		&mFullScreen,									"key=f"													);
	mParams->addButton( "Screen shot",		bind(& PointCloudGpuApp::screenShot, this ),	"key=s"													);
	mParams->addButton( "Quit",				bind(& PointCloudGpuApp::quit, this ),			"key=q"													);
	mParams->addSeparator();
	mParams->addText( "CAMERA" );
	mParams->addParam( "Eye point",			&mEyePoint																								);
	mParams->addParam( "Look at",			&mLookAt																								);
	mParams->addParam( "Rotation",			&mRotation																								);
	mParams->addSeparator();
	mParams->addText( "USER" );
	mParams->addParam( "Depth",				&mDepth,										"min=0.0 max=2000.0 step=1.0 keyIncr=Z keyDecr=z"		);
	mParams->addParam( "Point size",		&mPointSize,									"min=0.025 max=100.000 step=0.001 keyIncr=P keyDecr=p"	);
	mParams->addParam( "Remove background",	&mRemoveBackground,								"key=b"													);
	mParams->addParam( "Scale",				&mScale																									);

}

void PointCloudGpuApp::shutdown()
{
	mDevice->stop();
}

// Runs update logic
void PointCloudGpuApp::update()
{
	mFrameRate = getAverageFps();

	if ( mFullScreen != mFullScreenPrev ) {
		setFullScreen( mFullScreen );
		mFullScreenPrev = mFullScreen;
	}

	if (mRemoveBackground != mRemoveBackgroundPrev)
	{
		mDevice->removeBackground(mRemoveBackground);
		mRemoveBackgroundPrev = mRemoveBackground;
	}

	if ( !mDevice->isCapturing() ) {
		if ( getElapsedFrames() % 90 == 0 ) {
			mDevice->start();
		}
	}
	
	for ( vector<Skeleton>::const_iterator iter = mSkeletons.begin(); iter != mSkeletons.end(); ++iter ) {
		if ( iter->size() == (uint32_t)JointName::NUI_SKELETON_POSITION_COUNT ) {

			// Look at spine
			Vec3f spine = iter->at( JointName::NUI_SKELETON_POSITION_SPINE ).getPosition() * -mEyePoint.z;
			mLookAt.x = spine.x;
			mLookAt.y = spine.y;
			mEyePoint.x = -mLookAt.x * 0.25f;
			mEyePoint.y = -mLookAt.y * 0.125f;
		}
	}

	mCamera.lookAt( mEyePoint, mLookAt );
}

CINDER_APP_BASIC( PointCloudGpuApp, RendererGl )

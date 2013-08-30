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
class MeshApp : public ci::app::AppBasic 
{
public:
	void								draw();
	void								keyDown( ci::app::KeyEvent event );
	void								prepareSettings( ci::app::AppBasic::Settings* settings );
	void								resize();
	void								shutdown();
	void								setup();
	void								update();
private:
	float								mBrightTolerance;
	KinectSdk::KinectRef				mKinect;
	float								mMeshUvMix;
	bool								mRemoveBackground;
	bool								mRemoveBackgroundPrev;
	ci::Vec3f							mScale;
	bool								mShowColor;
	std::vector<KinectSdk::Skeleton>	mSkeletons;
	ci::gl::TextureRef					mTextureColor;
	ci::gl::TextureRef					mTextureDepth;
	float								mColorOffsetX;
	float								mColorOffsetY;
	void								onFrame( KinectSdk::Frame frame, const KinectSdk::DeviceOptions& deviceOptions );
	
	void								initMesh();
	ci::gl::VboMeshRef					mVboMesh;
	ci::gl::GlslProgRef					mShader;

	ci::CameraPersp						mCamera;
	ci::Vec3f							mEyePoint;
	ci::Vec3f							mLookAt;
	ci::Vec3f							mRotation;

	ci::ColorAf							mLightAmbient;
	ci::ColorAf							mLightDiffuse;
	ci::Vec3f							mLightPosition;
	float								mLightShininess;
	ci::ColorAf							mLightSpecular;

	float								mFrameRate;
	bool								mFullScreen;
	bool								mFullScreenPrev;

	ci::params::InterfaceGlRef			mParams;

	void								screenShot();
};

using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

const int32_t	kMeshHeight	= 240;
const int32_t	kMeshWidth	= 320;

void MeshApp::draw()
{
	gl::clear( ColorAf::black(), true );
	gl::setMatrices( mCamera );
	gl::color( ColorAf::white() );

	if ( mTextureDepth && mTextureColor && mVboMesh ) {
		gl::pushMatrices();
		gl::scale( -1.0f, -1.0f, -1.0f );
		gl::rotate( mRotation );

		mTextureDepth->bind( 0 );
		mTextureColor->bind( 1 );
		
		mShader->bind();
		mShader->uniform( "brightTolerance",	mBrightTolerance 						);
		mShader->uniform( "eyePoint",		mEyePoint 								);
		mShader->uniform( "lightAmbient",	mLightAmbient 							);
		mShader->uniform( "lightDiffuse",	mLightDiffuse 							);
		mShader->uniform( "lightPosition",	mLightPosition 							);
		mShader->uniform( "lightSpecular",	mLightSpecular 							);
		mShader->uniform( "positions",		0 										);
		mShader->uniform( "scale",			mScale 									);
		mShader->uniform( "showColor",		mShowColor 								);
		mShader->uniform( "shininess",		mLightShininess 						);
		mShader->uniform( "Color",			1 										);
		mShader->uniform( "ColorOffset",		Vec2f( mColorOffsetX, mColorOffsetY )	);
		mShader->uniform( "uvmix",			mMeshUvMix								);

		gl::draw( mVboMesh );

		mShader->unbind();
		mTextureDepth->unbind();
		mTextureColor->unbind();
		gl::popMatrices();
	}

	mParams->draw();
}

void MeshApp::initMesh()
{
	float heightf	= (float)kMeshHeight;
	float widthf	= (float)kMeshWidth;

	gl::VboMesh::Layout vboLayout;
	vector<uint32_t> vboIndices;
	vector<Vec3f> vboPositions;
	vector<Vec2f> vboTexCoords;

	vboLayout.setStaticIndices();
	vboLayout.setStaticPositions();
	vboLayout.setStaticTexCoords2d();

	vector<Vec2f> quad;
	quad.push_back( Vec2f( 0.0f, 0.0f ) );
	quad.push_back( Vec2f( 0.0f, 1.0f ) );
	quad.push_back( Vec2f( 1.0f, 0.0f ) );
	quad.push_back( Vec2f( 1.0f, 0.0f ) );
	quad.push_back( Vec2f( 0.0f, 1.0f ) );
	quad.push_back( Vec2f( 1.0f, 1.0f ) );

	for ( int32_t y = 0; y < kMeshHeight; ++y ) {
		for ( int32_t x = 0; x < kMeshWidth; ++x ) {
			float xf = (float)x;
			float yf = (float)y;

			vboPositions.push_back( Vec3f( xf - widthf * 0.5f, yf - heightf * 0.5f, 0.0f ) );

			vboTexCoords.push_back( Vec2f( xf / widthf, yf / heightf ) );

			if ( x < kMeshWidth && y < kMeshHeight ) {

				for ( vector<Vec2f>::const_iterator vertIt = quad.cbegin(); vertIt != quad.cend(); ++vertIt ) {
					xf = (float)x + vertIt->x;
					yf = (float)y + vertIt->y;

					vboIndices.push_back( (uint32_t)( xf + yf * widthf ) );
				}
			}
		}
	}

	mVboMesh = gl::VboMesh::create( vboPositions.size(), vboIndices.size(), vboLayout, GL_TRIANGLES );
	mVboMesh->bufferIndices( vboIndices );
	mVboMesh->bufferPositions( vboPositions );
	mVboMesh->bufferTexCoords2d( 0, vboTexCoords );
	mVboMesh->unbindBuffers();

	try {
		mShader = gl::GlslProg::create( loadResource( RES_SHADER_USER_VERT ), loadResource( RES_SHADER_USER_FRAG ) );
	} catch ( gl::GlslProgCompileExc ex ) {
		console() << "Unable to compile shader: " << ex.what() << endl;
		quit();
	}

	vboIndices.clear();
	vboPositions.clear();
	vboTexCoords.clear();
}

void MeshApp::keyDown( KeyEvent event )
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

void MeshApp::onFrame( Frame frame, const DeviceOptions& deviceOptions )
{
	if ( mTextureColor ) {
		mTextureColor->update( frame.getColorSurface(), frame.getColorSurface().getBounds() );
	} else {
		mTextureColor = gl::Texture::create( frame.getColorSurface() );
		mTextureColor->setWrap( GL_REPEAT, GL_REPEAT );
	}
	mTextureDepth->update( Surface32f( frame.getDepthSurface() ) );
	mSkeletons = frame.getSkeletons();
}

void MeshApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate( 60.0f );
}

void MeshApp::resize()
{
	mEyePoint	= Vec3f( 0.0f, 0.0f, 100.0f );
	mLookAt		= Vec3f::zero();
	mRotation	= Vec3f::zero();
	mCamera.lookAt( mEyePoint, mLookAt );
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 1.0f, 15000.0f );

	gl::enable( GL_DEPTH_TEST );
	glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
	gl::enable( GL_POLYGON_SMOOTH );
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableAlphaBlending();
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	gl::color( ColorAf::white() );
}

void MeshApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void MeshApp::setup()
{
	mKinect = Kinect::create();
	mKinect->connectEventHandler( &MeshApp::onFrame, this );
	mKinect->enableUserColor( false );
	mKinect->removeBackground();
	mKinect->start();

	mLightAmbient	= ColorAf( 0.0f, 0.0f, 0.0f, 1.0f );
	mLightDiffuse	= ColorAf( 0.5f, 0.5f, 0.5f, 1.0f );
	mLightPosition	= Vec3f( 0.0f, -600.0f, 180.0f );
	mLightShininess	= 2.0f;
	mLightSpecular	= ColorAf( 1.0f, 1.0f, 1.0f, 1.0f );

	mBrightTolerance		= 0.2f;
	mFrameRate				= 0.0f;
	mFullScreen				= isFullScreen();
	mFullScreenPrev			= mFullScreen;
	mMeshUvMix				= 0.2f;
	mRemoveBackground		= true;
	mRemoveBackgroundPrev	= mRemoveBackground;
	mScale					= Vec3f( 1.0f, 1.0f, 500.0f );
	mShowColor				= false;
	mColorOffsetX			= 0.0f;
	mColorOffsetY			= 0.0f;

	mParams = params::InterfaceGl::create( "Parameters", Vec2i( 250, 500 ) );
	mParams->addSeparator( "" );
	mParams->addParam( "Bright tolerance",	&mBrightTolerance,					"min=0.000 max=1.000 step=0.001 keyDecr=b keyIncr=B"		);
	mParams->addParam( "Remove background",	&mRemoveBackground,					"key=c"														);
	mParams->addParam( "Scale",				&mScale																							);
	mParams->addSeparator();
	mParams->addParam( "Eye point",			&mEyePoint																						);
	mParams->addParam( "Look at",			&mLookAt																						);
	mParams->addParam( "Rotation",			&mRotation																						);
	mParams->addSeparator();
	mParams->addParam( "Show Color",		&mShowColor,						"key=d"														);
	mParams->addParam( "Color offset X",	&mColorOffsetX,						"min=0.000 max=1.000 step=0.001 keyDecr=e keyIncr=E"		);
	mParams->addParam( "Color offset Y",	&mColorOffsetY,						"min=0.000 max=1.000 step=0.001 keyDecr=f keyIncr=F"		);
	mParams->addSeparator();
	mParams->addParam( "Light position",	&mLightPosition																					);
	mParams->addParam( "Light shininess",	&mLightShininess,					"min=0.000 max=10000.000 step=0.001 keyDecr=g keyIncr=G"	);
	mParams->addSeparator();
	mParams->addParam( "Frame rate",		&mFrameRate,						"", true													);
	mParams->addParam( "Full screen",		&mFullScreen,						"key=h"														);
	mParams->addButton( "Save screen shot",	bind(& MeshApp::screenShot, this ),	"key=space"													);
	mParams->addButton( "Quit",				bind(& MeshApp::quit, this ),		"key=esc"													);

	gl::Texture::Format format;
	format.setInternalFormat( GL_RGBA_FLOAT32_ATI );
	mTextureDepth = gl::Texture::create( Surface32f( kMeshWidth, kMeshHeight, false, SurfaceChannelOrder::RGBA ), format );

	initMesh();

	resize();
}

void MeshApp::shutdown()
{
	mKinect->stop();
}

void MeshApp::update()
{
	mFrameRate = getAverageFps();

	if ( mFullScreen != mFullScreenPrev ) {
		setFullScreen( mFullScreen );
		mFullScreenPrev = mFullScreen;
	}

	if ( mRemoveBackground != mRemoveBackgroundPrev ) {
		mKinect->removeBackground( mRemoveBackground );
		mRemoveBackgroundPrev = mRemoveBackground;
	}

	if ( mKinect->isCapturing() ) {
		mKinect->update();
		for ( vector<Skeleton>::const_iterator iter = mSkeletons.begin(); iter != mSkeletons.end(); ++iter ) {
			if ( iter->size() == (uint32_t)JointName::NUI_SKELETON_POSITION_COUNT ) {

				// Camera follow...
				Vec3f spine	= iter->at( JointName::NUI_SKELETON_POSITION_SPINE ).getPosition() * mEyePoint.z;
				mLookAt.x	= spine.x * 0.05f;
				mLookAt.y	= spine.y * 0.05f;
				mEyePoint.x = -mLookAt.x * 0.5f;
				mEyePoint.y = -mLookAt.y * 0.25f;
			}
		}
	} else {
		if ( getElapsedFrames() % 90 == 0 ) {
			mKinect->start();
		}
	}

	mCamera.lookAt( mEyePoint, mLookAt );
}

CINDER_APP_BASIC( MeshApp, RendererGl )

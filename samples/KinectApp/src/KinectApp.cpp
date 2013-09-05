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

#include <algorithm>
#include "boost/algorithm/string.hpp"
#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "Kinect.h"

/*
* This application explores the features of the Kinect SDK wrapper. It 
* demonstrates how to start a device, query for devices, adjust tilt, 
* and read and represent color, depth, and skeleton data.  
* It's also useful as a device test and control panel.
*/
class KinectApp : public ci::app::AppBasic 
{
public:
	void								draw();
	void								prepareSettings( ci::app::AppBasic::Settings* settings );
	void								setup();
	void								shutdown();
	void								update();
private:
	bool								mCapture;
	bool								mCapturePrev;
	bool								mBinaryMode;
	bool								mBinaryModePrev;
	bool								mEnabledColor;
	bool								mEnabledColorPrev;
	bool								mEnabledDepth;
	bool								mEnabledDepthPrev;
	bool								mEnabledNearMode;
	bool								mEnabledNearModePrev;
	bool								mEnabledSeatedMode;
	bool								mEnabledSeatedModePrev;
	bool								mEnabledSkeletons;
	bool								mEnabledSkeletonsPrev;
	bool								mEnabledStats;
	bool								mFlipped;
	bool								mFlippedPrev;
	bool								mInverted;
	bool								mInvertedPrev;

	// Kinect
	ci::Surface8u						mColorSurface;
	ci::Surface16u						mDepthSurface;
	int32_t								mDeviceCount;
	KinectSdk::DeviceOptions			mDeviceOptions;
	KinectSdk::KinectRef				mKinect;
	std::vector<KinectSdk::Skeleton>	mSkeletons;
	int32_t								mTilt;
	int32_t								mTiltPrev;
	int32_t								mUserCount;
	void								onFrame( KinectSdk::Frame frame, const KinectSdk::DeviceOptions& deviceOptions );
	void								startKinect();

	ci::CameraPersp						mCamera;

	float								mFrameRateApp;
	float								mFrameRateDevice;
	bool								mFullScreen;
	ci::params::InterfaceGlRef			mParams;
	bool								mRemoveBackground;
	bool								mRemoveBackgroundPrev;
	void								resetStats();

	void								screenShot();
};

using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;

void KinectApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::gray( 0.1f ) );

	if ( mKinect->isCapturing() ) {
		gl::setMatrices( mCamera );

		// Draw skeletons
		gl::pushMatrices();
		gl::scale( Vec2f::one() * 0.5f );
		gl::translate( 0.0f, -0.62f, 0.0f );

		uint32_t i = 0;
		for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.begin(); skeletonIt != mSkeletons.end(); ++skeletonIt, i++ ) {
			gl::color( mKinect->getUserColor( i ) );
			for ( Skeleton::const_iterator boneIt = skeletonIt->begin(); boneIt != skeletonIt->end(); ++boneIt ) {
				const Bone& bone	= boneIt->second;
				Vec3f position		= bone.getPosition();
				Vec3f destination	= skeletonIt->at( bone.getStartJoint() ).getPosition();

				gl::drawLine( position, destination );
				gl::drawSphere( position, 0.025f, 16 );
			}
		}

		// Draw depth and color textures
		gl::popMatrices();
		gl::setMatricesWindow( getWindowSize(), true );

		gl::color( Colorf::white() );
		if ( mDepthSurface ) {
			Area srcArea( 0, 0, mDepthSurface.getWidth(), mDepthSurface.getHeight() );
			Rectf destRect( 265.0f, 15.0f, 505.0f, 195.0f );
			gl::draw( gl::Texture( mDepthSurface ), srcArea, destRect );
		}
		if ( mColorSurface ) {
			Area srcArea( 0, 0, mColorSurface.getWidth(), mColorSurface.getHeight() );
			Rectf destRect( 508.0f, 15.0f, 748.0f, 195.0f );
			gl::draw( gl::Texture( mColorSurface ), srcArea, destRect );
		}
	}

	mParams->draw();
}

void KinectApp::onFrame( Frame frame, const DeviceOptions& deviceOptions )
{
	mColorSurface	= frame.getColorSurface();
	mDepthSurface	= frame.getDepthSurface();
	mSkeletons		= frame.getSkeletons();
}

void KinectApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1005, 570 );
	settings->setFrameRate( 60.0f );
}

void KinectApp::resetStats()
{
	mFrameRateDevice	= 0.0f;
	mUserCount			= 0;
}

void KinectApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void KinectApp::setup()
{
	glLineWidth( 2.0f );
	gl::color( ColorAf::white() );

	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 3.0f ), Vec3f::zero() );
	mCamera.setPerspective( 45.0f, getWindowAspectRatio(), 1.0f, 1000.0f );

	mBinaryMode				= false;
	mBinaryModePrev			= mBinaryMode;
	mCapture				= true;
	mCapturePrev			= mCapture;
	mDeviceCount			= 0;
	mEnabledColor			= true;
	mEnabledColorPrev		= mEnabledColor;
	mEnabledDepth			= true;
	mEnabledDepthPrev		= mEnabledDepth;
	mEnabledNearMode		= false;
	mEnabledNearModePrev	= mEnabledNearMode;
	mEnabledSeatedMode		= false;
	mEnabledSeatedModePrev	= mEnabledSeatedMode;
	mEnabledSkeletons		= true;
	mEnabledSkeletonsPrev	= mEnabledSkeletons;
	mEnabledStats			= true;
	mFlipped				= false;
	mFlippedPrev			= mFlipped;
	mFrameRateApp			= 0.0f;
	mFrameRateDevice		= 0.0f;
	mFullScreen				= isFullScreen();
	mInverted				= false;
	mInvertedPrev			= mInverted;
	mRemoveBackground		= false;
	mRemoveBackgroundPrev	= mRemoveBackground;
	mUserCount				= 0;

	mKinect = Kinect::create();
	mKinect->connectEventHandler( &KinectApp::onFrame, this );
	startKinect();
	
	mParams = params::InterfaceGl::create( "Parameters", Vec2i( 245, 500 ) );
	mParams->addText( "DEVICE" );
	mParams->addParam( "Device count",			&mDeviceCount,							"", true				);
	mParams->addParam( "Device angle",			&mTilt,									"min=-" + 
		toString( Kinect::MAXIMUM_TILT_ANGLE ) + " max=" + toString( Kinect::MAXIMUM_TILT_ANGLE ) + " step=1"	);
	mParams->addSeparator();
	mParams->addText( "STATISTICS");
	mParams->addParam( "Collect statistics",	&mEnabledStats,							"key=t"					);
	mParams->addParam( "App frame rate",		&mFrameRateApp,							"", true				);
	mParams->addParam( "Device frame rate",		&mFrameRateDevice,						"", true				);
	mParams->addParam( "User count",			&mUserCount,							"", true				);
	mParams->addSeparator();
	mParams->addText( "CAPTURE" );
	mParams->addParam( "Capture",				&mCapture,								"key=c" 				);
	mParams->addParam( "Depth",					&mEnabledDepth,							"key=d" 				);
	mParams->addParam( "Skeletons",				&mEnabledSkeletons,						"key=k" 				);
	mParams->addParam( "Color",					&mEnabledColor,							"key=v" 				);
	mParams->addSeparator();
	mParams->addText( "INPUT");
	mParams->addParam( "Remove background",		&mRemoveBackground,						"key=b" 				);
	mParams->addParam( "Binary depth mode",		&mBinaryMode,							"key=w" 				);
	mParams->addParam( "Invert binary image",	&mInverted,								"key=i" 				);
	mParams->addParam( "Flip input",			&mFlipped,								"key=m" 				);
	mParams->addParam( "Near mode",				&mEnabledNearMode,						"key=n" 				);
	mParams->addParam( "Seated mode",			&mEnabledSeatedMode,					"key=e" 				);
	mParams->addSeparator();
	mParams->addText( "APPLICATION" );
	mParams->addParam( "Full screen",			&mFullScreen,							"key=f"					);
	mParams->addButton( "Screen shot",			bind( &KinectApp::screenShot, this ),	"key=s"					);
	mParams->addButton( "Quit",					bind( &KinectApp::quit, this ),			"key=q"				);
}

void KinectApp::shutdown()
{
	mKinect->stop();
}

void KinectApp::startKinect()
{
	mDeviceCount = Kinect::getDeviceCount();
	
	mDeviceOptions.enableDepth( mEnabledDepth );
	mDeviceOptions.enableNearMode( mEnabledNearMode );
	mDeviceOptions.enableSkeletonTracking( mEnabledSkeletons, mEnabledSeatedMode );
	mDeviceOptions.enableColor( mEnabledColor );
	mKinect->enableBinaryMode( mBinaryMode );
	mKinect->removeBackground( mRemoveBackground );
	mKinect->setFlipped( mFlipped );

	if ( mKinect->isCapturing() ) {
		mKinect->stop();
	}

	try {
		mKinect->start( mDeviceOptions );
	} catch ( Kinect::ExcDeviceCreate ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect::ExcDeviceInit ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect::ExcDeviceInvalid ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect::ExcOpenStreamColor ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect::ExcOpenStreamDepth ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect::ExcSkeletonTrackingEnable ex ) {
		console() << ex.what() << endl;
	}
	
	console() << "Device ID: " << mKinect->getDeviceOptions().getDeviceId() << endl;

	mTilt = mKinect->getTilt();
	mTiltPrev = mTilt;

	resetStats();
}

void KinectApp::update()
{
	mFrameRateApp = getAverageFps();

	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
	}

	if ( mRemoveBackground != mRemoveBackgroundPrev ) {
		mKinect->removeBackground( mRemoveBackground );
		mRemoveBackgroundPrev = mRemoveBackground;
	}

	if ( mFlipped != mFlippedPrev ) {
		mKinect->setFlipped( mFlipped );
		mFlippedPrev = mFlipped;
	}

	if ( mCapture != mCapturePrev ) {
		mCapturePrev = mCapture;
		if ( mCapture ) {
			startKinect();
		} else {
			mKinect->stop();
		}
	}

	if ( mEnabledColor		!= mEnabledColorPrev		|| 
		mEnabledDepth		!= mEnabledDepthPrev		|| 
		mEnabledNearMode	!= mEnabledNearModePrev		|| 
		mEnabledSeatedMode	!= mEnabledSeatedModePrev	|| 
		mEnabledSkeletons	!= mEnabledSkeletonsPrev	) {
		startKinect();
		mEnabledColorPrev		= mEnabledColor;
		mEnabledDepthPrev		= mEnabledDepth;
		mEnabledNearModePrev	= mEnabledNearMode;
		mEnabledSeatedModePrev	= mEnabledSeatedMode;
		mEnabledSkeletonsPrev	= mEnabledSkeletons;
	}

	if ( mBinaryMode	!= mBinaryModePrev	|| 
		mInverted		!= mInvertedPrev ) {
		mKinect->enableBinaryMode( mBinaryMode, mInverted );
		mBinaryModePrev	= mBinaryMode;
		mInvertedPrev	= mInverted;
	}

	if ( mKinect->isCapturing() ) {
		mKinect->update();

		if ( mTilt != mTiltPrev ) {
			mKinect->setTilt( mTilt );
			mTiltPrev = mTilt;
		}

		if ( mEnabledStats ) {
			mUserCount			= mKinect->getUserCount();
			mFrameRateDevice	= mKinect->getFrameRate();
			
		} else {
			resetStats();
		}

	} else {
		if ( getElapsedFrames() % 90 == 0 ) {
			mKinect->start();
		}
	}
}

CINDER_APP_BASIC( KinectApp, RendererGl )

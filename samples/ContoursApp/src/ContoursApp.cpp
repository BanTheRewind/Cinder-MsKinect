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
#include "cinder/gl/Texture.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "ContourFinder.h"
#include "Kinect.h"
#include "Particle.h"

/*
* This application demonstrates how to combine Kinect input
* with OpenCV to track user contours.
*/
class ContoursApp : public ci::app::AppBasic 
{
public:
	void					draw();
	void					keyDown( ci::app::KeyEvent event );
	void					prepareSettings( ci::app::AppBasic::Settings* settings );
	void					resize();
	void					shutdown();
	void					setup();
	void					update();
private:
	MsKinect::DeviceRef		mDevice;
	void					onFrame( MsKinect::Frame frame, const MsKinect::DeviceOptions& deviceOptions );

	std::vector<Particle>	mParticles;

	ci::Channel16u			mChannel;
	ContourFinderRef		mContourFinder;
	std::vector<Contour>	mContours;

	bool					mFullScreen;
	bool					mFullScreenPrev;
	
	void					screenShot();
};

using namespace ci;
using namespace ci::app;
using namespace MsKinect;
using namespace std;

const float kPointSpacing		= 10.0f;
const float kInteractiveForce	= 0.4f;
const float kInteractiveRadius	= 80.0f;
const float kFloatMax			= numeric_limits<float>::max();

void ContoursApp::draw()
{
	gl::clear( ColorAf::black() );

	gl::begin( GL_POINTS );
	for ( vector<Particle>::const_iterator iter = mParticles.begin(); iter != mParticles.end(); ++iter ) {
		gl::color( iter->getColor() );
		gl::vertex( iter->getPosition() );
	}
	gl::end();
	
}

void ContoursApp::keyDown( KeyEvent event )
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

void ContoursApp::onFrame( Frame frame, const DeviceOptions& deviceOptions )
{
	mChannel = frame.getDepthSurface().getChannelRed();
}

void ContoursApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
	settings->setFullScreen( true );
}

void ContoursApp::resize()
{
	gl::enable( GL_POINT_SMOOTH );
	glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
	glPointSize( 3.0f );
	gl::enableAlphaBlending();

	gl::setViewport( getWindowBounds() );
	gl::setMatricesWindow( getWindowSize() );
}

void ContoursApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void ContoursApp::setup()
{
	mDevice = Device::create();
	mDevice->connectEventHandler( &ContoursApp::onFrame, this );
	mDevice->removeBackground();
	mDevice->enableBinaryMode( true );

	try {
		mDevice->start( DeviceOptions().enableColor( false ).enableSkeletonTracking( false ).setDepthResolution( ImageResolution::NUI_IMAGE_RESOLUTION_80x60 ) );
	} catch ( Device::ExcDeviceCreate ex ) {
		console() << ex.what() << endl;
	} catch ( Device::ExcDeviceInit ex ) {
		console() << ex.what() << endl;
	} catch ( Device::ExcDeviceInvalid ex ) {
		console() << ex.what() << endl;
	} catch ( Device::ExcOpenStreamDepth ex ) {
		console() << ex.what() << endl;
	}
	
	resize();

	mFullScreen		= false;
	mFullScreenPrev = mFullScreen;

	float height	= (float)getWindowHeight();
	float width		= (float)getWindowWidth();
	Colorf color	= Colorf::white();
	Vec2f position	= Vec2f::one() * kPointSpacing * 0.5f;
	while ( position.y < height ) {
		while ( position.x < width ) {
			position.x	+= kPointSpacing;
			color.r		= position.x / width;
			color.g		= 1.0f - color.r;
			color.b		= position.y / height;
			Particle particle( position, color );
			mParticles.push_back( particle );
		}
		position.x = kPointSpacing * 0.5f;
		position.y += kPointSpacing * 1.5f;
	}

	mContourFinder = ContourFinder::create();
}

void ContoursApp::shutdown()
{
	mDevice->stop();
}

void ContoursApp::update()
{
	if ( mFullScreen != mFullScreenPrev ) {
		setFullScreen( mFullScreen );
		mFullScreenPrev = mFullScreen;
	}

	if ( mDevice->isCapturing() ) {
		mDevice->update();
		if ( mChannel ) {
			
			// Find contours
			mContours = mContourFinder->findContours( Channel8u( mChannel ) );
			
			// Scale contours to window
			Vec2f scale = Vec2f( getWindowSize() ) / Vec2f( mChannel.getSize() );
			for ( vector<Contour>::iterator contourIt = mContours.begin(); contourIt != mContours.end(); ++contourIt ) {
				for ( vector<Vec2f>::iterator pointIt = contourIt->getPoints().begin(); pointIt != contourIt->getPoints().end(); ++pointIt ) {
					pointIt->operator*=( scale );
				}
				contourIt->calcCentroid();
			}
		}
	} else {
		if ( getElapsedFrames() % 90 == 0 ) {
			mDevice->start();
		}

	}

	for ( vector<Particle>::iterator partIt = mParticles.begin(); partIt != mParticles.end(); ++partIt ) {
		Particle& particle = *partIt;

		float x = particle.getPosition().x;
		float y = particle.getPosition().y;

		for ( vector<Contour>::const_iterator contourIt = mContours.begin(); contourIt != mContours.end(); ++contourIt ) {
			const Contour& contour = *contourIt;

			// Iterate through outline to determine if particle is inside 
			// contour and which point in outline is the closest
			bool isInsideContour	= false;
			float minDistance		= kFloatMax;
			Vec2f closestPoint( kFloatMax, kFloatMax );

			Vec2f centroid			= contour.getCentroid();
			uint32_t count			= contour.getPoints().size();
			uint32_t j				= count - 1;
			for ( uint32_t i = 0; i < count; i++ ) {
					
				ci::Vec2f a	= contour.getPoints().at( i );
				ci::Vec2f b	= contour.getPoints().at( j );
				float x1	= a.x;
				float y1	= a.y;
				float x2	= b.x;
				float y2	= b.y;
					
				// Apply ray-cast algorithm to determine if point is inside contour
				if ( ( ( y1 < y && y2 >= y ) || 
						( y2 < y && y1 >= y) ) && 
					( x1 <= x || x2 <= x ) ) {
					isInsideContour ^= ( x1 + ( y - y1 ) / ( y2 - y1 ) * ( x2 - x1 ) < x );
				}
				j = i;
					
				// Find closest point
				float distance = particle.getPosition().xy().distanceSquared( a );
				if ( distance < minDistance ) {
					closestPoint = a;
					minDistance = distance;
				}
					
			}

			// Set particle acceleration away from center
			Vec2f dir		= particle.getPosition().xy() - centroid;
			float dist = particle.getPosition().xy().distance( centroid );
			if ( dist < kInteractiveRadius ) {
				float amp = ( 1.0f - ( dist / kInteractiveRadius ) ) * kInteractiveForce;
				particle.addAcceleration( dir * amp );
			}

			// Particle is inside contour
			dir				= closestPoint - particle.getPosition().xy();
			float distance	= dir.length();
			if ( isInsideContour ) {
					
				// Set particle acceleration towards closest point in outline
				distance = math<float>::max( kInteractiveRadius - distance, 0.0f );
				if ( distance > 0.0f ) {
					float amp	= ( distance / kInteractiveRadius ) * kInteractiveForce;
					particle.addAcceleration( dir * amp );
				}
					
			}
		}

		partIt->update();
	}
}

CINDER_APP_BASIC( ContoursApp, RendererGl )

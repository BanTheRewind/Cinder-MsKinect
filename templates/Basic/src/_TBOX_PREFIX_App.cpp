#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"

#include "Kinect.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class _TBOX_PREFIX_App : public AppBasic 
{
public:
	void draw();
	void keyDown( KeyEvent event );
	void prepareSettings( ci::app::AppBasic::Settings* settings );
	void setup();
	void shutdown();
	void update();
private:
	ci::CameraPersp		mCamera;
	MsKinect::DeviceRef	mDevice;
	vector<Vec3f>		mPoints;
};

// Kinect image size
const Vec2i	kKinectSize( 640, 480 );

void _TBOX_PREFIX_App::draw()
{
	gl::clear();
	gl::setMatrices( mCamera );

	// Draw point cloud
	gl::begin( GL_POINTS );
	for ( auto iter = mPoints.begin(); iter != mPoints.end(); ++iter ) {
		float depth = 1.0f - iter->z / mCamera.getEyePoint().z * -1.5f;
		gl::color( ColorAf( 1.0f, depth, 1.0f - depth, depth ) );
		gl::vertex( *iter );
	}
	gl::end();
}

void _TBOX_PREFIX_App::keyDown( KeyEvent event )
{
	switch( event.getCode() ) {
		case KeyEvent::KEY_ESCAPE:
			quit();
		break;
		case KeyEvent::KEY_f:
			setFullScreen( ! isFullScreen() );
		break;
	}
}

void _TBOX_PREFIX_App::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 1024, 768 );
}

// Set up
void _TBOX_PREFIX_App::setup()
{
	// Set up OpenGL
	gl::enable( GL_DEPTH_TEST );
	glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
	glEnable( GL_POINT_SMOOTH );
	glPointSize( 0.25f );
	gl::enableAlphaBlending();
	gl::enableAdditiveBlending();

	// Start Kinect with isolated depth tracking only
	mDevice = MsKinect::Device::create();
	mDevice->start( MsKinect::DeviceOptions().enableSkeletonTracking( false ).enableVideo( false ).setDepthResolution( MsKinect::ImageResolution::NUI_IMAGE_RESOLUTION_640x480 ) );

	// Set up camera
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 670.0f ), Vec3f::zero() );
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 0.01f, 5000.0f );
}

void _TBOX_PREFIX_App::shutdown()
{
	mDevice->stop();
}

void _TBOX_PREFIX_App::update()
{
	if ( mDevice->isCapturing() ) {
		Vec3f offset( Vec2f( kKinectSize ) * Vec2f( -0.5f, 0.5f ) );
		offset.z = mCamera.getEyePoint().z;
		Vec3f position = Vec3f::zero();
		mPoints.clear();

		// Iterate image rows
		for ( int32_t y = 0; y < kKinectSize.y; ++y ) {
			for ( int32_t x = 0; x < kKinectSize.x; ++x ) {
				// Read depth as 0.0 - 1.0 float
				float depth = mDevice->getDepthAt( Vec2i( x, y ) );

				// Add position to point list
				position.z = depth * mCamera.getEyePoint().z * -3.0f;
				mPoints.push_back( position * Vec3f( 1.1f, -1.1f, 1.0f ) + offset );

				// Shift point
				++position.x;
			}

			// Update position
			position.x = 0.0f;
			++position.y;
		}
	} 
	else {
		// If Kinect initialization failed, try again every 90 frames
		if ( getElapsedFrames() % 90 == 0 ) {
			mDevice->start();
		}
	}
}

CINDER_APP_BASIC( _TBOX_PREFIX_App, RendererGl )

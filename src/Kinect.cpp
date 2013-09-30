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

#include "Kinect.h"

#include "cinder/app/App.h"
#include "cinder/Utilities.h"	

#include <comutil.h>

namespace MsKinect
{

using namespace ci;
using namespace ci::app;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////

const double kTiltRequestInterval = 1.5;

Matrix44f toMatrix44f( const Matrix4& m ) 
{
	return Matrix44f( Vec4f( m.M11, m.M12, m.M13, m.M14 ), 
		Vec4f( m.M21, m.M22, m.M23, m.M24 ), 
		Vec4f( m.M31, m.M32, m.M33, m.M34 ), 
		Vec4f( m.M41, m.M42, m.M43, m.M44 ) );
}
Quatf toQuatf( const Vector4& v ) 
{
	return Quatf( v.w, v.x, v.y, v.z );
}
Vec3f toVec3f( const Vector4& v ) 
{
	return Vec3f( v.x, v.y, v.z );
}

void CALLBACK deviceStatus( long hr, const WCHAR *instanceName, const WCHAR *deviceId, void *data )
{
	Device* device = reinterpret_cast<Device*>( data );
	if ( SUCCEEDED( hr ) ) {
		device->start( device->getDeviceOptions() );
	} else {
		device->error( hr );
		reinterpret_cast<Device*>( data )->stop();
	}
}

const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformNone			= { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformDefault		= { 0.5f, 0.5f, 0.5f, 0.05f, 0.04f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformSmooth		= { 0.5f, 0.1f, 0.5f, 0.1f, 0.1f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformVerySmooth	= { 0.7f, 0.3f, 1.0f, 1.0f, 1.0f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformMax			= { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformParams[ 5 ]	= 
{ kTransformNone, kTransformDefault, kTransformSmooth, kTransformVerySmooth, kTransformMax };

//////////////////////////////////////////////////////////////////////////////////////////////

Bone::Bone( const Vector4& position, const _NUI_SKELETON_BONE_ORIENTATION& bone )
{
	mAbsRotQuat	= toQuatf( bone.absoluteRotation.rotationQuaternion );
	mAbsRotMat	= toMatrix44f( bone.absoluteRotation.rotationMatrix );
	mJointEnd	= bone.endJoint;
	mJointStart	= bone.startJoint;
	mPosition	= toVec3f( position );
	mRotQuat	= toQuatf( bone.hierarchicalRotation.rotationQuaternion );
	mRotMat		= toMatrix44f( bone.hierarchicalRotation.rotationMatrix );
}

const Quatf& Bone::getAbsoluteRotation() const 
{ 
	return mAbsRotQuat; 
}
const Matrix44f& Bone::getAbsoluteRotationMatrix() const 
{ 
	return mAbsRotMat; 
}
JointName Bone::getEndJoint() const
{
	return mJointEnd;
}
const Vec3f& Bone::getPosition() const 
{ 
	return mPosition; 
}
const Quatf& Bone::getRotation() const 
{ 
	return mRotQuat; 
}
const Matrix44f& Bone::getRotationMatrix() const 
{ 
	return mRotMat; 
}
JointName Bone::getStartJoint() const
{
	return mJointStart;
}

//////////////////////////////////////////////////////////////////////////////////////////////

DeviceOptions::DeviceOptions()
{
	mDeviceId					= "";
	mDeviceIndex				= 0;
	mEnabledColor				= true;
	mEnabledDepth				= true;
	mEnabledFrameSync			= false;
	mEnabledNearMode			= false;
	mEnabledSeatedMode			= false;
	mEnabledSkeletonTracking	= true;
	mEnabledUserTracking		= true;
	setColorResolution( ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
	setDepthResolution( ImageResolution::NUI_IMAGE_RESOLUTION_320x240 );
}

DeviceOptions& DeviceOptions::enableDepth( bool enable )
{
	mEnabledDepth = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableFrameSync( bool enable )
{
	mEnabledFrameSync = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableNearMode( bool enable )
{
	mEnabledNearMode = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableSkeletonTracking( bool enable, bool seatedMode )
{
	mEnabledSeatedMode			= seatedMode;
	mEnabledSkeletonTracking	= enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableUserTracking( bool enable )
{
	mEnabledUserTracking = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableColor( bool enable )
{
	mEnabledColor = enable;
	return *this;
}
	
ImageResolution DeviceOptions::getColorResolution() const
{
	return mColorResolution;
}

const Vec2i& DeviceOptions::getColorSize() const 
{
	return mColorSize;
}

ImageResolution DeviceOptions::getDepthResolution() const
{
	return mDepthResolution;
}
	
const Vec2i& DeviceOptions::getDepthSize() const
{
	return mDepthSize;
}

const string& DeviceOptions::getDeviceId() const
{
	return mDeviceId;
}
	
int32_t DeviceOptions::getDeviceIndex() const
{
	return mDeviceIndex;
}

bool DeviceOptions::isDepthEnabled() const
{
	return mEnabledDepth;
}

bool DeviceOptions::isFrameSyncEnabled() const
{
	return mEnabledFrameSync;
}
	
bool DeviceOptions::isNearModeEnabled() const
{
	return mEnabledNearMode;
}
	
bool DeviceOptions::isSeatedModeEnabled() const
{
	return mEnabledSeatedMode;
}

bool DeviceOptions::isSkeletonTrackingEnabled() const
{
	return mEnabledSkeletonTracking;
}
	
bool DeviceOptions::isUserTrackingEnabled() const
{
	return mEnabledUserTracking;
}

bool DeviceOptions::isColorEnabled() const
{
	return mEnabledColor;
}

DeviceOptions& DeviceOptions::setColorResolution( const ImageResolution& resolution )
{
	mColorResolution = resolution;
	switch ( mColorResolution ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_1280x960:
		mColorSize = Vec2i( 1280, 960 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mColorSize = Vec2i( 640, 480 );
		break;
	default:
		mColorResolution = NUI_IMAGE_RESOLUTION_INVALID;
		mColorSize = Vec2i::zero();
		mEnabledColor = false;
		break;
	}
	return *this;
}

DeviceOptions& DeviceOptions::setDepthResolution( const ImageResolution& resolution )
{
	mDepthResolution = resolution;
	switch ( mDepthResolution ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mDepthSize					= Vec2i( 640, 480 );
		mEnabledUserTracking		= false;
		mEnabledSkeletonTracking	= false;
		mEnabledSeatedMode			= false;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		mDepthSize = Vec2i( 320, 240 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		mDepthSize = Vec2i( 80, 60 );
		break;
	default:
		mDepthResolution = NUI_IMAGE_RESOLUTION_INVALID;
		mDepthSize = Vec2i::zero();
		mEnabledDepth = false;
		break;
	}
	return *this;
}

DeviceOptions& DeviceOptions::setDeviceId( const std::string& id )
{
	mDeviceId = id;
	return *this;
}
	
DeviceOptions& DeviceOptions::setDeviceIndex( int32_t index )
{
	mDeviceIndex = index;
	return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Frame::Frame()
	: mId( 0 )
{
}

Frame::Frame( long id, const Surface8u& color, const Surface16u& depth, const vector<Skeleton>& skeletons )
	: mColorSurface( color ), mDepthSurface( depth ), mId( id ), mSkeletons( skeletons )
{
}

const Surface8u& Frame::getColorSurface() const
{
	return mColorSurface;
}

const Surface16u& Frame::getDepthSurface() const
{
	return mDepthSurface;
}

long Frame::getId() const
{
	return mId;
}

const vector<Skeleton>&	Frame::getSkeletons() const
{
	return mSkeletons;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Device::ColorFrame::ColorFrame( const ci::Surface8u& surface, __int64 timeStamp )
: mSurface( surface ), mTimeStamp( timeStamp )
{
}

const Surface8u& Device::ColorFrame::getSurface() const
{
	return mSurface;
}

__int64 Device::ColorFrame::getTimeStamp() const
{
	return mTimeStamp;
}

//////////////////////////////////////////////////////////////////////////////////////////////

vector<Colorf> Device::sUserColors = getUserColors();

ci::Colorf Device::getUserColor( uint32_t id ) 
{ 
	return sUserColors.at( ci::math<uint32_t>::clamp( id, 0, 5 ) ); 
}

vector<Colorf> Device::getUserColors()
{
	if ( sUserColors.size() == NUI_SKELETON_COUNT ) {
		return sUserColors;
	}

	vector<Colorf> colors;
	colors.push_back( Colorf( 0.0f, 1.0f, 1.0f ) );
	colors.push_back( Colorf( 0.0f, 0.0f, 1.0f ) );
	colors.push_back( Colorf( 0.0f, 1.0f, 0.0f ) );
	colors.push_back( Colorf( 0.0f, 0.5f, 1.0f ) );
	colors.push_back( Colorf( 0.0f, 1.0f, 0.5f ) );
	colors.push_back( Colorf( 0.0f, 0.5f, 0.5f ) );
	return colors;
}

DeviceRef Device::create()
{
	return DeviceRef( new Device( ) );
}

Device::Device()
{
	NuiSetDeviceStatusCallback( &MsKinect::deviceStatus, this );
	init();
	for ( int32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {
		mSkeletons.push_back( Skeleton() );
	}

	App::get()->getSignalUpdate().connect( bind( &Device::update, this ) );
}

Device::~Device()
{
	mEventHandler = nullptr;

	stop();
	if ( mSensor != 0 ) {
		mSensor->NuiShutdown();
		if ( mSensor ) {
			mSensor->Release();
			mSensor = 0;
			mDepthStreamHandle = 0;
			mColorStreamHandle = 0;
		}
	}
}

void Device::connectEventHandler( const function<void ( Frame, const DeviceOptions& )>& eventHandler )
{
	mEventHandler = eventHandler;
}

void Device::deactivateUsers()
{
	for ( uint32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {
		mActiveUsers[ i ] = false;
	}
}

void Device::enableBinaryMode( bool enable, bool invertImage )
{
	mBinary = enable;
	mInverted = invertImage;
}

void Device::enableUserColor( bool enable )
{
	mGreyScale = !enable;
}

void Device::enableVerbose( bool enable )
{
	mVerbose = enable;
}

void Device::error( long hr ) {
	if ( !mVerbose ) {
		return;
	}
	switch ( hr ) {
	case E_POINTER:
		console() << "Bad pointer.";
		break;
	case E_INVALIDARG:
		console() << "Invalid argument.";
		break;
	case E_NUI_DEVICE_NOT_READY:
		console() << "Device not ready.";
		break;
	case E_NUI_FEATURE_NOT_INITIALIZED:
		console() << "Feature not initialized.";
		break;
	case E_NUI_NOTCONNECTED:
		console() << "Unable to connect to device.";
		break;
	case E_FAIL:
		console() << "Attempt failed.";
		break;
	case E_NUI_IMAGE_STREAM_IN_USE:
		console() << "Image stream already in use.";
		break;
	case E_NUI_FRAME_NO_DATA:
		console() << "No frame data available";
		break;
	case E_OUTOFMEMORY:
		console() << "Out of memory (maximum number of Kinect devices may have been reached).";
		break;
	case ERROR_TOO_MANY_CMDS:
		console() << "Too many commands sent. Angle change requests must be made at least 1.5s apart.";
		break;
	case ERROR_RETRY:
		console() << "Device is busy.  Retry in a moment.";
		break;
	case S_FALSE:
		console() << "Data not available.";
	case S_OK:
		break;
	default:
		console() << "Unknown error (Code " + toString( hr ) + ")";
	}
	console() << endl;
}

Vec2i Device::getColorDepthPos( const ci::Vec2i& v )
{
	long x;
	long y;
	mSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( mDeviceOptions.getColorResolution(), mDeviceOptions.getDepthResolution(), 0, v.x, v.y, mDepthSurface.getPixel( v ).r, &x, &y ); 
	return Vec2i( (int32_t)x, (int32_t)y );
}

float Device::getDepthAt( const ci::Vec2i& pos ) const
{
	float depthNorm		= 0.0f;
	if ( mDepthSurface ) {
		uint16_t depth	= 0x10000 - mDepthSurface.getPixel( pos ).r;
		depth			= depth << 2;
		depthNorm		= 1.0f - (float)depth / 65535.0f;
	}
	return depthNorm;
}
	
int32_t Device::getDeviceCount()
{
	int32_t deviceCount = 0;
	NuiGetSensorCount( &deviceCount );
	return deviceCount;
}

const DeviceOptions& Device::getDeviceOptions() const
{
	return mDeviceOptions;
}

float Device::getFrameRate() const 
{
	return mFrameRate;
}

Quatf Device::getOrientation() const
{
	Vector4 v;
	mSensor->NuiAccelerometerGetCurrentReading( &v );
	return toQuatf( v );
}

Vec2i Device::getSkeletonDepthPos( const ci::Vec3f& position )
{
	float x;
	float y;
	Vector4 pos;
	pos.x = position.x;
	pos.y = position.y;
	pos.z = position.z;
	pos.w = 1.0f;
	NuiTransformSkeletonToDepthImage( pos, &x, &y, mDeviceOptions.getDepthResolution() );
	return Vec2i( (int32_t)x, (int32_t)y );
}

Vec2i Device::getSkeletonColorPos( const ci::Vec3f& position )
{
	float x;
	float y;
	Vector4 pos;
	pos.x = position.x;
	pos.y = position.y;
	pos.z = position.z;
	pos.w = 1.0f;
	NuiTransformSkeletonToDepthImage( pos, &x, &y, mDeviceOptions.getColorResolution() );
	return Vec2i( (int32_t)x, (int32_t)y );
}

int32_t Device::getTilt()
{
	long degrees = 0L;
	if ( mCapture && mSensor != 0 ) {
		long hr = mSensor->NuiCameraElevationGetAngle( &degrees );
		if ( FAILED( hr ) ) {
			console() << "Unable to retrieve device angle:" << endl;
			error( hr );
		}
	}
	return (int32_t)degrees;
}

int_fast8_t Device::getTransform() const
{
	return mTransform;
}

int32_t Device::getUserCount()
{
	return mDeviceOptions.isDepthEnabled() ? mUserCount : 0;
}

void Device::init( bool reset )
{
	// Only set these when first initializing the device
	if ( !reset ) {
		mBinary				= false;
		mFlipped			= false;
		mGreyScale			= false;
		mInverted			= false;
		mRemoveBackground	= false;
		mTransform			= TRANSFORM_DEFAULT;
		mVerbose			= true;
	}

	mCapture				= false;
	mColorEvent				= 0;
	mColorStreamHandle		= 0;
	mDepthStreamHandle		= 0;
	mDepthEvent				= 0;
	mEventHandler			= nullptr;
	mFrameId				= 0;
	mFrameRate				= 0.0f;
	mNewColorSurface		= false;
	mNewDepthSurface		= false;
	mNewSkeletons			= false;
	mIsSkeletonDevice		= false;
	mReadTime				= 0.0;
	mRgbColor				= 0;
	mRgbDepth				= 0;
	mSensor					= 0;
	mSkeletonEvent			= 0;
	mTiltRequestTime		= 0.0;
	mUserCount				= 0;
	
	deactivateUsers();
}

bool Device::isCapturing() const 
{
	return mCapture; 
}

bool Device::isFlipped() const 
{ 
	return mFlipped; 
}

long Device::openColorStream()
{
	if ( mSensor != 0 ) {
		long hr = mSensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, mDeviceOptions.getColorResolution(), 0, 2, mColorEvent, &mColorStreamHandle );
		if ( FAILED( hr ) ) {
			console() << "Unable to open color image stream: " << endl;
			error( hr );
			stop();
			return hr;
		}
	}
	return S_OK;
}

long Device::openDepthStream()
{
	if ( mSensor != 0) {
		NUI_IMAGE_TYPE type = mDeviceOptions.getDepthResolution() != ImageResolution::NUI_IMAGE_RESOLUTION_640x480 && 
			HasSkeletalEngine( mSensor ) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH;
		long hr = mSensor->NuiImageStreamOpen( type, mDeviceOptions.getDepthResolution(), 0, 2, mDepthEvent, &mDepthStreamHandle );
		if ( FAILED( hr ) ) {
			console() << "Unable to open depth image stream: " << endl;
			error( hr );
			stop();
			return hr;
		}
	}
	return S_OK;
}

void Device::pixelToDepthSurface( uint16_t *buffer )
{
	int32_t height	= mDepthSurface.getHeight();
	int32_t width	= mDepthSurface.getWidth();
	int32_t size	= width * height * 6; // 6 is 3 color channels * sizeof( uint16_t )

	Pixel16u* rgbRun	= mRgbDepth;
	uint16_t* bufferRun	= buffer;

	if ( mFlipped ) {
		for ( int32_t y = 0; y < height; ++y ) {
			for ( int32_t x = 0; x < width; ++x ) {
				bufferRun		= buffer + ( y * width + ( ( width - x ) - 1 ) );
				rgbRun			= mRgbDepth + ( y * width + x );
				*rgbRun			= shortToPixel( *bufferRun );
			}
		}
	} else {
		for ( int32_t i = 0; i < width * height; ++i ) {
			Pixel16u pixel = shortToPixel( *bufferRun );
			++bufferRun;
			*rgbRun = pixel;
			++rgbRun;
		}
	}

	memcpy( mDepthSurface.getData(), mRgbDepth, size );
}

void Device::pixelToColorSurface( uint8_t *buffer )
{
	int32_t height	= mColorSurface.getHeight();
	int32_t width	= mColorSurface.getWidth();
	int32_t size	= width * height * 4;
	
	if ( mFlipped ) {
		uint8_t *flipped = new uint8_t[ size ];
		for ( int32_t y = 0; y < height; ++y ) {
			for ( int32_t x = 0; x < width; ++x ) {
				int32_t dest	= ( y * width + x ) * 4;
				int32_t src		= ( y * width + ( ( width - x ) - 1 ) ) * 4;
				for ( int32_t i = 0; i < 4; ++i ) {
					flipped[ dest + i ] = buffer[ src + i ];
				}
			}
		}
		memcpy( mColorSurface.getData(), flipped, size );
		delete [] flipped;
	} else {
		memcpy( mColorSurface.getData(), buffer, size );
	}
}

void Device::removeBackground( bool remove )
{
	mRemoveBackground = remove;
}

void Device::run()
{
	HANDLE events[ 4 ];
    events[ 0 ] = mColorEvent;
    events[ 1 ] = mDepthEvent;
    events[ 2 ] = mSkeletonEvent;

	while ( mCapture ) {
		if ( mSensor != 0 ) {
			double time = getElapsedSeconds();

			WaitForMultipleObjects( sizeof( events ) / sizeof( events[ 0 ]), events, 0, WAIT_TIME );

			const NUI_IMAGE_FRAME* frameColor	= 0;
			const NUI_IMAGE_FRAME* frameDepth	= 0;
			NUI_SKELETON_FRAME frameSkeleton	= { 0 };

			bool readColor		= !mNewColorSurface;
			bool readDepth		= !mNewDepthSurface;
			bool readSkeleton	= !mNewSkeletons;
			if ( mDeviceOptions.isFrameSyncEnabled() && mDeviceOptions.isColorEnabled() && mDeviceOptions.isDepthEnabled() ) {
				if ( readColor != readDepth ) {
					readColor	= false;
					readDepth	= false;
				}
				readSkeleton	= readDepth;
			}
			readColor		= readColor && mDeviceOptions.isColorEnabled();
			readDepth		= readDepth && mDeviceOptions.isDepthEnabled();
			readSkeleton	= readSkeleton && mDeviceOptions.isSkeletonTrackingEnabled();

			//////////////////////////////////////////////////////////////////////////////////////////////

			if ( readDepth && WAIT_OBJECT_0 == WaitForSingleObject( mDepthEvent, 0 ) ) {
				if ( SUCCEEDED( NuiImageStreamGetNextFrame( mDepthStreamHandle, 0, &frameDepth ) ) && 
					frameDepth != 0 && frameDepth->pFrameTexture != 0 ) {
					mDepthTimeStamp				= frameDepth->liTimeStamp.QuadPart;
					INuiFrameTexture* texture	= frameDepth->pFrameTexture;
					_NUI_LOCKED_RECT lockedRect;
					long hr = texture->LockRect( 0, &lockedRect, 0, 0 );
					if ( FAILED( hr ) ) {
						error( hr );
					}
					if ( lockedRect.Pitch == 0 ) {
						console() << "Invalid buffer length received" << endl;
					} else {
						pixelToDepthSurface( (uint16_t*)lockedRect.pBits );
					}

					hr = NuiImageStreamReleaseFrame( mDepthStreamHandle, frameDepth );
					if ( FAILED( hr ) ) {
						error( hr ); 
					}
					
					mUserCount = 0;
					for ( uint32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {
						if ( mActiveUsers[ i ] ) {
							++mUserCount;
						}
					}
					mNewDepthSurface = true;
				}
			}

			//////////////////////////////////////////////////////////////////////////////////////////////
				
			if ( readColor && WAIT_OBJECT_0 == WaitForSingleObject( mColorEvent, 0 ) ) {
				if ( SUCCEEDED( NuiImageStreamGetNextFrame( mColorStreamHandle, 0, &frameColor ) ) && 
					frameColor != 0 && frameColor->pFrameTexture != 0 ) {
					INuiFrameTexture* texture = frameColor->pFrameTexture;
					_NUI_LOCKED_RECT lockedRect;
					long hr = texture->LockRect( 0, &lockedRect, 0, 0 );
					if ( FAILED( hr ) ) {
						error( hr );
					}
					if ( lockedRect.Pitch != 0 ) {
						pixelToColorSurface( (uint8_t*)lockedRect.pBits );
						if ( mDeviceOptions.isFrameSyncEnabled() ) {
							mColorFrames.push_back( ColorFrame( mColorSurface, frameColor->liTimeStamp.QuadPart ) );
							if ( mColorFrames.size() > 10 ) {
								mColorFrames.erase( mColorFrames.begin() );
							}
						}

					} else {
						console() << "Invalid buffer length received." << endl;
					}

					hr = NuiImageStreamReleaseFrame( mColorStreamHandle, frameColor );
					if ( FAILED( hr ) ) {
						error( hr );
					}
					mNewColorSurface = true;
				}
			}

			//////////////////////////////////////////////////////////////////////////////////////////////

			if ( readSkeleton && WAIT_OBJECT_0 == WaitForSingleObject( mSkeletonEvent, 0 ) ) {
				long hr = NuiSkeletonGetNextFrame( 0, &frameSkeleton );
				if ( SUCCEEDED( hr ) ) {
					bool foundSkeleton = false;
					for ( int32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {

						mSkeletons.at( i ).clear();

						NUI_SKELETON_TRACKING_STATE trackingState = frameSkeleton.SkeletonData[ i ].eTrackingState;
						if ( trackingState == NUI_SKELETON_TRACKED || trackingState == NUI_SKELETON_POSITION_ONLY ) {

							if ( !foundSkeleton ) {
								_NUI_TRANSFORM_SMOOTH_PARAMETERS transform = kTransformParams[ mTransform ];
								hr = mSensor->NuiTransformSmooth( &frameSkeleton, &transform );
								if ( FAILED( hr ) ) {
									error( hr );
								}
								foundSkeleton = true;
							}

							if ( mFlipped ) {
								( frameSkeleton.SkeletonData + i )->Position.x *= -1.0f;
								for ( int32_t j = 0; j < (int32_t)NUI_SKELETON_POSITION_COUNT; ++j ) {
									( frameSkeleton.SkeletonData + i )->SkeletonPositions[ j ].x *= -1.0f;
								}
							}

							_NUI_SKELETON_BONE_ORIENTATION bones[ NUI_SKELETON_POSITION_COUNT ];
							hr = NuiSkeletonCalculateBoneOrientations( frameSkeleton.SkeletonData + i, bones );
							if ( FAILED( hr ) ) {
								error( hr );
							}

							for ( int32_t j = 0; j < (int32_t)NUI_SKELETON_POSITION_COUNT; ++j ) {
								Bone bone( *( ( frameSkeleton.SkeletonData + i )->SkeletonPositions + j ), *( bones + j ) );
								( mSkeletons.begin() + i )->insert( std::pair<JointName, Bone>( (JointName)j, bone ) );
							}

						}

					}
					mNewSkeletons = true;
				}

				mFrameRate	= (float)( 1.0 / ( time - mReadTime ) );
				mReadTime	= time;
			}
		}
	}
	return;
}

void Device::setFlipped( bool flipped ) 
{
	mFlipped = flipped;
}

void Device::setTilt( int32_t degrees )
{
	double elapsedSeconds = getElapsedSeconds();
	if ( mCapture && mSensor != 0 && elapsedSeconds - mTiltRequestTime > kTiltRequestInterval ) {
		long hr = mSensor->NuiCameraElevationSetAngle( (long)math<int32_t>::clamp( degrees, -MAXIMUM_TILT_ANGLE, MAXIMUM_TILT_ANGLE ) );
		if ( FAILED( hr ) ) {
			console() << "Unable to change device angle: " << endl;
			error( hr );
		}
		mTiltRequestTime = elapsedSeconds;
	}
}

void Device::setTransform( int_fast8_t transform )
{
	mTransform = transform;
}

Device::Pixel16u Device::shortToPixel( uint16_t value )
{
	uint16_t depth	= 0xFFFF - 0x10000 * ( ( value&  0xFFF8 ) >> 3 ) / 0x0FFF;
	uint16_t user	= value&  7;

	Pixel16u pixel;
	pixel.b = 0;
	pixel.g = 0;
	pixel.r = 0;

	if ( user > 0 && user < 7 ) {
		mActiveUsers[ user - 1 ] = true;
	}

	if ( mBinary ) {
		uint16_t backgroundColor	= mInverted ? 0xFFFF : 0;
		uint16_t userColor			= mInverted ? 0 : 0xFFFF;

		if ( user == 0 || user == 7 ) {
			pixel.r = pixel.g = pixel.b = mRemoveBackground ? backgroundColor : userColor;
		} else {
			pixel.r = pixel.g = pixel.b = userColor;
		}
	} else if ( mGreyScale ) {
		if ( user == 0 || user == 7 ) {
			pixel.r = mRemoveBackground ? 0 : depth;
		} else {
			pixel.r = depth;
		}
		pixel.g = pixel.r;
		pixel.b = pixel.g;
	} else {
		switch ( user ) {
		case 0:
			if ( !mRemoveBackground ) {
				pixel.r = depth / 4;
				pixel.g = pixel.r;
				pixel.b = pixel.g;
			}
			break;
		case 1:
			pixel.r = depth;
			break;
		case 2:
			pixel.r = depth;
			pixel.g = depth;
			break;
		case 3:
			pixel.r = depth;
			pixel.b = depth;
			break;
		case 4:
			pixel.r = depth;
			pixel.g = depth / 2;
			break;
		case 5:
			pixel.r = depth;
			pixel.b = depth / 2;
			break;
		case 6:
			pixel.r = depth;
			pixel.g = depth / 2;
			pixel.b = pixel.g;
			break;
		case 7:
			if ( !mRemoveBackground ) {
				pixel.r = 0xFFFF - ( depth / 2 );
				pixel.g = pixel.r;
				pixel.b = pixel.g;
			}
		}
	}
	pixel.r = 0xFFFF - pixel.r;
	pixel.g = 0xFFFF - pixel.g;
	pixel.b = 0xFFFF - pixel.b;

	return pixel;
}

void Device::start( const DeviceOptions& deviceOptions ) 
{
	if ( !mCapture ) {
		mDeviceOptions	= deviceOptions;
		string deviceId	= mDeviceOptions.getDeviceId();
		int32_t index	= mDeviceOptions.getDeviceIndex();

		if ( index >= 0 ) {
			index = math<int32_t>::clamp( index, 0, math<int32_t>::max( getDeviceCount() - 1, 0 ) );
		}

		if ( mColorEvent == 0 ) {
			mColorEvent = CreateEvent( 0, 1, 0, 0 );
		}
		if ( mDepthEvent == 0 ) {
			mDepthEvent = CreateEvent( 0, 1, 0, 0 );
		}
		if ( mSkeletonEvent == 0 ) {
			mSkeletonEvent = CreateEvent( 0, 1, 0, 0 );
		}

		long hr = S_OK;
		if ( index >= 0 ) {
			hr = NuiCreateSensorByIndex( index, &mSensor );
			if ( FAILED( hr ) ) {
				console() << "Unable to create device instance " + toString( index ) + ": " << endl;
				error( hr );
				return;
			}
		} else if ( deviceId.length() > 0 ) {
			_bstr_t id = deviceId.c_str();
			hr = NuiCreateSensorById( id, &mSensor );
			if ( FAILED( hr ) ) {
				console() << "Unable to create device instance " + deviceId + ":" << endl;
				error( hr );
				return;
			}
		} else {
			console() << "Invalid device name or index." << endl;
			return;
		}

		hr = mSensor != 0 ? mSensor->NuiStatus() : E_NUI_NOTCONNECTED;
		if ( hr == E_NUI_NOTCONNECTED ) {
			error( hr );
			return;
		}

		if ( mSensor != 0 ) {
			mDeviceOptions.setDeviceIndex( mSensor->NuiInstanceIndex() );
			BSTR id = ::SysAllocString( mSensor->NuiDeviceConnectionId() ); 
			_bstr_t idStr( id );
			if ( idStr.length() > 0 ) {
				std::string str( idStr );
				deviceId = str;
			}
			::SysFreeString( id );
		} else {
			index = -1;
			deviceId = "";
		}

		mDeviceOptions.setDeviceIndex( index );
		mDeviceOptions.setDeviceId( deviceId );

		unsigned long flags;
		if ( !mDeviceOptions.isUserTrackingEnabled() ) {
			flags = NUI_INITIALIZE_FLAG_USES_DEPTH;
		} else {
			flags = NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
			if ( mDeviceOptions.isSkeletonTrackingEnabled() ) {
				flags |= NUI_INITIALIZE_FLAG_USES_SKELETON;
				if ( mDeviceOptions.isNearModeEnabled() ) {
					flags |= NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE;
				}
			}
		}
		if ( mDeviceOptions.isColorEnabled() ) {
			flags |= NUI_INITIALIZE_FLAG_USES_COLOR;
		}
		hr = mSensor->NuiInitialize( flags );
		if ( FAILED( hr ) ) {
			console() << "Unable to initialize device " + mDeviceOptions.getDeviceId() + ":" << endl;
			error( hr );
			throw ExcDeviceInit( hr, mDeviceOptions.getDeviceId() );
		}
		
		if ( mDeviceOptions.getColorResolution() != ImageResolution::NUI_IMAGE_RESOLUTION_INVALID ) {
			const Vec2i& videoSize = mDeviceOptions.getColorSize();
			if ( mDeviceOptions.isColorEnabled() ) {
				hr = openColorStream();
				if ( FAILED( hr ) ) {
					throw ExcOpenStreamColor( hr );
				}
			}
			mColorSurface	= Surface8u( videoSize.x, videoSize.y, false, SurfaceChannelOrder::BGRX );
			mRgbColor		= new Pixel[ videoSize.x * videoSize.y * 4 ];
		}

		if ( mDeviceOptions.getDepthResolution() != ImageResolution::NUI_IMAGE_RESOLUTION_INVALID ) {
			const Vec2i& depthSize = mDeviceOptions.getDepthSize();
			if ( mDeviceOptions.isDepthEnabled() ) {
				hr = openDepthStream();
				if ( FAILED( hr ) ) {
					throw ExcOpenStreamDepth( hr );
				}
			}
			mDepthSurface	= Surface16u( depthSize.x, depthSize.y, false, SurfaceChannelOrder::RGB );
			mRgbDepth		= new Pixel16u[ depthSize.x * depthSize.y * 3 ];
		}

		if ( mDeviceOptions.isSkeletonTrackingEnabled() && HasSkeletalEngine( mSensor ) ) {
			flags = NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE;
			if ( mDeviceOptions.isSeatedModeEnabled() ) {
				flags |= NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT;
			}
			hr = mSensor->NuiSkeletonTrackingEnable( mSkeletonEvent, flags );
			if ( FAILED( hr ) ) {
				console() << "Unable to initialize skeleton tracking for device " + mDeviceOptions.getDeviceId() + ": " << endl;
				error( hr );
				return;
			}
			mIsSkeletonDevice = true;
		}

		flags = NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM;
		if ( mDeviceOptions.isNearModeEnabled() ) {
			flags |= NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE | NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES;
		}
		hr = mSensor->NuiImageStreamSetImageFrameFlags( mDepthStreamHandle, flags );
		if ( FAILED( hr ) ) {
			console() << "Unable to set image frame flags: " << endl;
			error( hr );
		}

		mSkeletons.clear();
		for ( int32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {
			mSkeletons.push_back( Skeleton() );
		}

		// Start threads
		mCapture = true;
		mThread = shared_ptr<thread>( new thread( bind( &Device::run, this ) ) );
	}
}

void Device::stop()
{
	mCapture = false;
	if ( mThread ) {
		mThread->join();
		mThread.reset();
	}

	if ( mColorEvent != 0 ) {
		CloseHandle( mColorEvent );
		mColorEvent = 0;
	}
	if ( mDepthEvent != 0 ) {
		CloseHandle( mDepthEvent );
		mDepthEvent = 0;
	}
	if ( mSkeletonEvent != 0 ) {
		CloseHandle( mSkeletonEvent );
		mSkeletonEvent = 0;
	}

	if ( mRgbDepth != 0 ) {
		delete [] mRgbDepth;
	}
	if ( mRgbColor != 0 ) {
		delete [] mRgbColor;
	}
	init( true );
}

void Device::update()
{
	bool newFrame = true;
	if ( mDeviceOptions.isColorEnabled() ) {
		newFrame = newFrame && mNewColorSurface;
	}
	if ( mDeviceOptions.isDepthEnabled() ) {
		newFrame = newFrame && mNewDepthSurface;
	}
	if ( mDeviceOptions.isSkeletonTrackingEnabled() ) {
		newFrame = newFrame && mNewSkeletons;
	}
	if ( newFrame && mEventHandler != nullptr ) {
		if ( mDeviceOptions.isFrameSyncEnabled() && mDeviceOptions.isColorEnabled() && mDeviceOptions.isDepthEnabled() ) {
			list<ColorFrame>::const_iterator colorFrame = mColorFrames.begin();
			__int64 d0									= math<__int64>::abs( mDepthTimeStamp - colorFrame->getTimeStamp() );
			for ( list<ColorFrame>::const_iterator iter = mColorFrames.begin(); iter != mColorFrames.end(); ++iter ) {
				if ( colorFrame != iter ) {
					__int64 d1 = math<__int64>::abs( mDepthTimeStamp - iter->getTimeStamp() );
					if ( d0 < d1 ) {
						d0 = d1;
						colorFrame = iter;
					}
				}
			}

			Frame frame( mFrameId, colorFrame->getSurface(), mDepthSurface, mSkeletons );
			mEventHandler( frame, mDeviceOptions );
		} else {
			Frame frame( mFrameId, mColorSurface, mDepthSurface, mSkeletons );
			mEventHandler( frame, mDeviceOptions );
		}
		++mFrameId;
	}
	mNewColorSurface	= false;
	mNewDepthSurface	= false;
	mNewSkeletons		= false;
}

//////////////////////////////////////////////////////////////////////////////////////////////

const char* Device::Exception::what() const throw() 
{ 
	return mMessage; 
}

Device::ExcDeviceCreate::ExcDeviceCreate( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to create device. ID or index: %s. Error: %i", id, hr );
}

Device::ExcDeviceInit::ExcDeviceInit( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to initialize device. ID or index: %s. Error: %i", id, hr );
}

Device::ExcDeviceInvalid::ExcDeviceInvalid( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Invalid device ID or index: %s. Error: %i", id, hr );
}

Device::ExcOpenStreamColor::ExcOpenStreamColor( long hr )
{
	sprintf( mMessage, "Unable to open color stream. Error: %i", hr );
}

Device::ExcOpenStreamDepth::ExcOpenStreamDepth( long hr )
{
	sprintf( mMessage, "Unable to open depth stream. Error: %i", hr );
}

Device::ExcSkeletonTrackingEnable::ExcSkeletonTrackingEnable( long hr )
{
	sprintf( mMessage, "Unable to enable skeleton tracking. Error: %i", hr );
}
}
 
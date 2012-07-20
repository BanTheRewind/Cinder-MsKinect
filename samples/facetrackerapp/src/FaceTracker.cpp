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

#include "FaceTracker.h"

using namespace ci;
using namespace KinectSdk;
using namespace std;

FaceTrackerRef FaceTracker::create() 
{
	return FaceTrackerRef();
}

FaceTracker::FaceTracker()
{
	mCenter				= Vec2f::zero();
	mConfigDepth		= new FT_CAMERA_CONFIG();
	mConfigVideo		= new FT_CAMERA_CONFIG();
	mDepthImage			= 0;
	mFaceTracker		= 0;
	mFaceTrackerResult	= 0;
	mHint[ 0 ]			= FT_VECTOR3D();
	mHint[ 1 ]			= FT_VECTOR3D();
	mLastTrackSucceeded	= false;
	mTracking			= false;
	mVideoImage			= 0;
}

FaceTracker::~FaceTracker()
{
	if ( mConfigDepth ) {
		delete mConfigDepth;
	}
	if ( mConfigVideo ) {
		delete mConfigVideo;
	}
	if ( mDepthImage ) {
		mDepthImage->Release();
		mDepthImage = 0;
	}
	if ( mFaceTracker ) {
		mFaceTracker->Release();
		mFaceTracker = 0;
	}
	if ( mFaceTrackerResult ) {
		mFaceTrackerResult->Release();
		mFaceTrackerResult = 0;
	}
	if ( mVideoImage ) {
		mVideoImage->Release();
		mVideoImage = 0;
	}
}

bool FaceTracker::findFace( const Skeleton& skeleton, const Surface8u &videoSurface, const Surface16u &depthSurface )
{
	if ( !mTracking || skeleton.empty() || !videoSurface || !depthSurface ) {
		return false;
	}

	memcpy( mDepthImage->GetBuffer(), depthSurface.getData(), mDepthImage->GetBufferSize() );
	memcpy( mVideoImage->GetBuffer(), videoSurface.getData(), mVideoImage->GetBufferSize() );

	tagPOINT offset;
	offset.x = 0;
	offset.y = 0;
	FT_SENSOR_DATA sensorData( mVideoImage, mDepthImage, 1.0f, &offset );
	
	const Vec3f& head = skeleton.at( NUI_SKELETON_POSITION_HEAD ).getPosition();
	const Vec3f& neck = skeleton.at( NUI_SKELETON_POSITION_SHOULDER_CENTER ).getPosition();
	mHint[ 0 ] = FT_VECTOR3D( head.x, head.y, head.z );
	mHint[ 1 ] = FT_VECTOR3D( neck.x, neck.y, neck.z );

	HRESULT hr = S_OK;;
	if ( mLastTrackSucceeded ) {
		hr = mFaceTracker->ContinueTracking( &sensorData, mHint, mFaceTrackerResult );
	} else {
        hr = mFaceTracker->StartTracking( &sensorData, 0, mHint, mFaceTrackerResult );
    }
    mLastTrackSucceeded = SUCCEEDED( hr ) && SUCCEEDED( mFaceTrackerResult->GetStatus() );

	mFace.clear();
	if ( mLastTrackSucceeded ) {
        
		float* shapeUnits = 0;
		size_t suCount;
		BOOL suConverged;
		hr = mFaceTracker->GetShapeUnits( 0, &shapeUnits, &suCount, &suConverged );
		if ( SUCCEEDED( hr ) ) {

			float *animUnits;
			size_t auCount;
			hr = mFaceTrackerResult->GetAUCoefficients( &animUnits, &auCount );
			if ( SUCCEEDED( hr ) ) {

				float scale;
				float rotation[ 3 ];
				float translation[ 3 ];
				hr = mFaceTrackerResult->Get3DPose( &scale, rotation, translation );
				if ( SUCCEEDED( hr ) ) {

					IFTModel* faceModel;
					HRESULT hr = mFaceTracker->GetFaceModel( &faceModel );

					size_t vertexCount = faceModel->GetVertexCount();
					if ( vertexCount > 0 ) {
						FT_VECTOR2D* data = new FT_VECTOR2D[ sizeof( FT_VECTOR2D ) * vertexCount ];
						
						hr = faceModel->GetProjectedShape( mConfigDepth, 1.0f, offset, shapeUnits, 
							faceModel->GetSUCount(), animUnits, auCount, scale, rotation, translation, 
							data, vertexCount );

						if ( SUCCEEDED( hr ) ) {
							tagPOINT* points = new tagPOINT[ sizeof( tagPOINT ) * vertexCount ];

							for ( size_t i = 0; i < vertexCount; ++i ) {
								points[ i ].x = long( data[ i ].x + 0.5f );
								points[ i ].y = long( data[ i ].y + 0.5f );
							}

							FT_TRIANGLE* triangles;
							size_t triangleCount;
							hr = faceModel->GetTriangles( &triangles, &triangleCount );
							if ( SUCCEEDED( hr ) ) {
						
								for ( uint32_t i = 0; i < triangleCount; ++i ) { 
									tagPOINT& point0 = points[ triangles[ i ].i ];
									tagPOINT& point1 = points[ triangles[ i ].j ];
									tagPOINT& point2 = points[ triangles[ i ].k ];
									mFace.push_back( Vec2f( (float)point0.x, (float)point0.y ) );
									mFace.push_back( Vec2f( (float)point1.x, (float)point1.y ) );
									mFace.push_back( Vec2f( (float)point2.x, (float)point2.y ) );
								}

								RECT rect;
								hr = mFaceTrackerResult->GetFaceRect( &rect );
								if ( SUCCEEDED( hr ) ) {
									mBounds.x1 = (float)rect.left;
									mBounds.y1 = (float)rect.top;
									mBounds.x2 = (float)rect.right - 1.0f;
									mBounds.y2 = (float)rect.bottom - 1.0f;
								}
							}
							delete [] points;
						}
						delete [] data;
					}

				}

			}

		}
    } else {
		mCenter = Vec2f::zero();
		mBounds = Rectf();
        mFaceTrackerResult->Reset();
    }
    
	setCenter();
	
	return mLastTrackSucceeded;
}

const Rectf& FaceTracker::getBounds() const
{
	return mBounds;
}

const Vec2f& FaceTracker::getCenter() const
{
	return mCenter;
}

const vector<Vec2f>& FaceTracker::getFace() const
{
	return mFace;
}

void FaceTracker::setCenter()
{
	Vec2f center( (float)mVideoImage->GetWidth() * 0.5f, (float)mVideoImage->GetHeight() * 0.5f );
	if ( mFaceTrackerResult != 0 ) {
		if ( SUCCEEDED( mFaceTrackerResult->GetStatus() ) ) {
			tagRECT rect;
			mFaceTrackerResult->GetFaceRect( &rect );
			center.set( ( rect.left + rect.right ) * 0.5f, ( rect.top + rect.bottom ) * 0.5f );
		}
		mCenter += Vec2f( 0.02f * ( center.x - mCenter.x ), 0.02f * ( center.y - mCenter.y ) );
	} else {
		mCenter = center;
	}
}

void FaceTracker::setup( const DeviceOptions &deviceOptions )
{

	float focalLength	= NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
	Vec2i size			= Vec2i::zero();
	if ( deviceOptions.isDepthEnabled() ) {
		switch ( deviceOptions.getDepthResolution() ) {
		case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
			size.set( 320, 240 );
			focalLength	*= 0.5f;
			break;
		case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
			size.set( 640, 480 );
			break;
		case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
			size.set( 80, 60 );
			focalLength *= 0.25f;
			break;
		}
	}

	return;
	( *mConfigDepth ).FocalLength	= focalLength;
	mConfigDepth->Height		= size.y;
	mConfigDepth->Width			= size.x;

	focalLength	= NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
	size		= Vec2i::zero();
	if ( deviceOptions.isVideoEnabled() ) {
		switch ( deviceOptions.getVideoResolution() ) {
		case ImageResolution::NUI_IMAGE_RESOLUTION_1280x960:
			size.set( 1280, 960 );
			focalLength *= 2.0f;
			break;
		case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
			size.set( 640, 480 );
			break;
		}
	}

	mConfigVideo->FocalLength	= focalLength;
	mConfigVideo->Height		= size.y;
	mConfigVideo->Width			= size.x;

	mFaceTracker = FTCreateFaceTracker();
	HRESULT hr = mFaceTracker->Initialize( mConfigVideo, mConfigDepth, 0, 0 );
	if ( FAILED( hr ) ) {
		return;
	}

	hr = mFaceTracker->CreateFTResult( &mFaceTrackerResult );
	if ( FAILED( hr ) || !mFaceTrackerResult ) {
		return;
	}

	mDepthImage = FTCreateImage();
	if ( !mDepthImage || FAILED( mDepthImage->Allocate( mConfigDepth->Width, mConfigDepth->Height, FTIMAGEFORMAT_UINT16_D13P3 ) ) ) {
		return;
	}

	mVideoImage = FTCreateImage();
	if ( !mVideoImage || FAILED( mVideoImage->Allocate( mConfigVideo->Width, mConfigVideo->Height, FTIMAGEFORMAT_UINT8_B8G8R8X8 ) ) ) {
		return;
	}

	setCenter();

	mTracking = true;

}

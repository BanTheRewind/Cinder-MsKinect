/*
* 
* Copyright (c) 2013, Ban the Rewind, Wieden+Kennedy
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
#include "cinder/app/App.h"

namespace MsKinect
{
using namespace ci;
using namespace ci::app;
using namespace std;

FaceTracker::Face::Face()
{
	mPoseMatrix.setToNull();
	mUserId = 0;
}

const FaceTracker::AnimationUnitMap& FaceTracker::Face::getAnimationUnits() const
{
	return mAnimationUnits;
}

const Rectf& FaceTracker::Face::getBounds() const
{
	return mBounds;
}

const TriMesh& FaceTracker::Face::getMesh() const
{
	return mMesh;
}

const TriMesh2d& FaceTracker::Face::getMesh2d() const
{
	return mMesh2d;
}

const Matrix44f& FaceTracker::Face::getPoseMatrix() const
{
	return mPoseMatrix;
}

size_t FaceTracker::Face::getUserId() const
{
	return mUserId;
}

//////////////////////////////////////////////////////////////////////////////////////////////

FaceTrackerRef FaceTracker::create()
{
	return FaceTrackerRef( new FaceTracker() );
}

FaceTracker::FaceTracker()
{
	mCalcMesh		= true;
	mCalcMesh2d		= true;
	mEventHandler	= nullptr;
	mFaceTracker	= 0;
	mModel			= 0;
	mNewFace		= false;
	mResult			= 0;
	mRunning		= false;
	mSuccess		= false;
	mUserId			= 0;
}

FaceTracker::~FaceTracker()
{
	stop();

	if ( mFaceTracker != 0 ) {
		mFaceTracker->Release();
		mFaceTracker = 0;
	}

	if ( mSensorData.pVideoFrame != 0 ) {
		mSensorData.pVideoFrame->Release();
		mSensorData.pVideoFrame = 0;
	}

	if ( mSensorData.pDepthFrame != 0 ) {
		mSensorData.pDepthFrame->Release();
		mSensorData.pDepthFrame = 0;
	}

	if ( mModel != 0 ) {
		mModel->Release();
		mModel = 0;
	}

	if ( mResult != 0 ) {
		mResult->Release();
		mResult = 0;
	}
}

IFTFaceTracker* FaceTracker::getFaceTracker() const
{
	return mFaceTracker;
}

IFTModel* FaceTracker::getModel() const
{
	return mModel;
}

IFTResult* FaceTracker::getResult() const
{
	return mResult;
}

void FaceTracker::enableCalcMesh( bool enabled )
{
	mCalcMesh = enabled;
}

void FaceTracker::enableCalcMesh2d( bool enabled )
{
	mCalcMesh2d = enabled;
}

bool FaceTracker::isCalcMeshEnabled() const
{
	return mCalcMesh;
}

bool FaceTracker::isCalcMesh2dEnabled() const
{
	return mCalcMesh2d;
}

bool FaceTracker::isTracking() const
{
	return mRunning;
}

void FaceTracker::start( const DeviceOptions& deviceOptions )
{
	stop();

	mConfigColor.Height				= deviceOptions.getColorSize().y;
	mConfigColor.Width				= deviceOptions.getColorSize().x;
	switch ( deviceOptions.getColorResolution() ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mConfigColor.FocalLength	= NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_1280x960:
		mConfigColor.FocalLength	= NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.0f;
		break;
	default:
		mConfigColor.FocalLength	= 0.0f;
		break;
	}

	mConfigDepth.Height				= deviceOptions.getDepthSize().y;
	mConfigDepth.Width				= deviceOptions.getDepthSize().x;
	switch ( deviceOptions.getDepthResolution() ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		mConfigDepth.FocalLength	= NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 0.25f;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		mConfigDepth.FocalLength	= NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mConfigDepth.FocalLength	= NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.0f;
		break;
	default:
		mConfigDepth.FocalLength	= 0.0f;
		break;
	}

	long hr = S_OK;
	mFaceTracker = FTCreateFaceTracker();
    if ( !mFaceTracker ) {
		throw ExcFaceTrackerCreate();
	}

	hr = mFaceTracker->Initialize( &mConfigColor, &mConfigDepth, 0, 0 );
	if ( FAILED( hr ) ) {
		throw ExcFaceTrackerInit( hr );
    }

	hr = mFaceTracker->CreateFTResult( &mResult );
	if ( FAILED( hr ) || mResult == 0 ) {
		throw ExcFaceTrackerCreateResult( hr );
	}

	tagPOINT offset;
	offset.x			= 0;
	offset.y			= 0;
	mSensorData.pDepthFrame	= FTCreateImage();
	mSensorData.pVideoFrame	= FTCreateImage();
	mSensorData.ViewOffset	= offset;
	mSensorData.ZoomFactor	= 1.0f;
	
	mRunning	= true;
	mThread		= ThreadRef( new thread( &FaceTracker::run, this ) );
}

void FaceTracker::stop()
{
	mRunning = false;
	if ( mThread ) {
		mThread->join();
		mThread.reset();
	}
}

void FaceTracker::update( const Surface8u& color, const Channel16u& depth, const Vec3f headPoints[ 2 ], size_t userId )
{
	if ( mNewFace && mEventHandler != nullptr ) {
		mEventHandler( mFace );
		if ( color && depth ) {
			mHeadPoints.clear();
			if ( headPoints != 0 ) {
				mHeadPoints.push_back( headPoints[ 0 ] );
				mHeadPoints.push_back( headPoints[ 1 ] );
			}

			bool attach = !mChannelDepth || !mSurfaceColor;

			mChannelDepth	= depth;
			mSurfaceColor	= color;
			mUserId			= userId;

			if ( attach ) {
				mSensorData.pVideoFrame->Attach( mSurfaceColor.getWidth(), mSurfaceColor.getHeight(), 
					(void*)mSurfaceColor.getData(), FTIMAGEFORMAT_UINT8_B8G8R8X8, mSurfaceColor.getWidth() * 4 );
				mSensorData.pDepthFrame->Attach( mChannelDepth.getWidth(), mChannelDepth.getHeight(), 
					(void*)mChannelDepth.getData(), FTIMAGEFORMAT_UINT16_D13P3,	mChannelDepth.getWidth() * 2 );
			}
		}
		mNewFace = false;
	}
}

void FaceTracker::run()
{
	while ( mRunning ) {
		if ( !mNewFace ) {

			long hr = S_OK;

			mFace.mAnimationUnits.clear();
			mFace.mBounds = Rectf( 0.0f, 0.0f, 0.0f, 0.0f );
			mFace.mMesh.clear();
			mFace.mMesh2d.clear();
			mFace.mPoseMatrix.setToIdentity();
			mFace.mUserId = mUserId;

			FT_VECTOR3D* hint = 0;
			if ( mHeadPoints.size() == 2 ) {
				hint = new FT_VECTOR3D[ 2 ];
				for ( size_t i = 0; i < 2; ++i ) {
					hint[ i ] = FT_VECTOR3D( mHeadPoints[ i ].x, mHeadPoints[ i ].y, mHeadPoints[ i ].z );
				}
			}

			if ( mSuccess ) {
				hr = mFaceTracker->ContinueTracking( &mSensorData, hint, mResult );
				console() << "Continue ";
			} else {
				hr = mFaceTracker->StartTracking( &mSensorData, 0, hint, mResult );
				console() << "Start ";
			}

			if ( hint != 0 ) {
				delete [] hint;
			}

			mSuccess = SUCCEEDED( hr ) && SUCCEEDED( mResult->GetStatus() );

			if ( mSuccess ) {
				console() << "succeeded";
			} else {
				console() << "failed";
			}
			console() << endl;
		
			if ( mSuccess ) {
				hr = mFaceTracker->GetFaceModel( &mModel );
				if ( SUCCEEDED( hr ) ) {
					float* shapeUnits		= 0;
					size_t numShapeUnits	= 0;
					int32_t haveConverged	= false;
					mFaceTracker->GetShapeUnits( 0, &shapeUnits, &numShapeUnits, &haveConverged );
							
					float* animationUnits;
					size_t numAnimationUnits;
					hr = mResult->GetAUCoefficients( &animationUnits, &numAnimationUnits );
					if ( SUCCEEDED( hr ) ) {
						for ( size_t i = 0; i < numAnimationUnits; ++i ) {
							mFace.mAnimationUnits[ (AnimationUnit)i ] = animationUnits[ i ];
						}
					}

					float scale;
					float rotation[ 3 ];
					float translation[ 3 ];
					hr = mResult->Get3DPose( &scale, rotation, translation );
					if ( SUCCEEDED( hr ) ) {
						Vec3f r( rotation[ 0 ], rotation[ 1 ], rotation[ 2 ] );
						Vec3f t( translation[ 0 ], translation[ 1 ], translation[ 2 ] );

						mFace.mPoseMatrix.translate( t );
						mFace.mPoseMatrix.rotate( r );
						mFace.mPoseMatrix.translate( -t );
						mFace.mPoseMatrix.translate( t );
						mFace.mPoseMatrix.scale( Vec3f::one() * scale );
					}

					size_t numVertices	= mModel->GetVertexCount();
							
					if ( numAnimationUnits > 0 && numShapeUnits > 0 && numVertices > 0 ) {
						if ( mCalcMesh ) {
							FT_VECTOR3D* pts = reinterpret_cast<FT_VECTOR3D*>( _malloca( sizeof( FT_VECTOR3D ) * numVertices ) );
							hr = mModel->Get3DShape( shapeUnits, numShapeUnits, animationUnits, numAnimationUnits, scale, rotation, translation, pts, numVertices );
							if ( SUCCEEDED( hr ) ) {
								for ( size_t i = 0; i < numVertices; ++i ) {
									Vec3f v( pts[ i ].x, pts[ i ].y, pts[ i ].z );
									mFace.mMesh.appendVertex( v );
								}

								FT_TRIANGLE* triangles;
								size_t triangleCount;
								hr = mModel->GetTriangles( &triangles, &triangleCount );
								if ( SUCCEEDED( hr ) ) {
									for ( size_t i = 0; i < triangleCount; ++i ) {
										mFace.mMesh.appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
									}
								}
							}
							_freea( pts );
						}

						if ( mCalcMesh2d ) {
							tagPOINT viewOffset	= { 0, 0 };
							FT_VECTOR2D* pts	= reinterpret_cast<FT_VECTOR2D*>( _malloca( sizeof( FT_VECTOR2D ) * numVertices ) );
							hr = mModel->GetProjectedShape( &mConfigColor, mSensorData.ZoomFactor, viewOffset, shapeUnits, numShapeUnits, animationUnits, 
								numAnimationUnits, scale, rotation, translation, pts, numVertices );
							if ( SUCCEEDED( hr ) ) {
								for ( size_t i = 0; i < numVertices; ++i ) {
									Vec2f v( pts[ i ].x + 0.5f, pts[ i ].y + 0.5f );
									mFace.mMesh2d.appendVertex( v );
								}

								FT_TRIANGLE* triangles;
								size_t triangleCount;
								hr = mModel->GetTriangles( &triangles, &triangleCount );
								if ( SUCCEEDED( hr ) ) {
									for ( size_t i = 0; i < triangleCount; ++i ) {
										mFace.mMesh2d.appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
									}
								}
							}
							_freea( pts );
						}
					}

					tagRECT rect;
					hr = mResult->GetFaceRect( &rect );
					if ( SUCCEEDED( hr ) ) {
						mFace.mBounds = Rectf( (float)rect.left, (float)rect.top, (float)rect.right, (float)rect.bottom );
					}
				}
			} else {
				mResult->Reset();
			}

			mNewFace = true;
		}
	}
}

void FaceTracker::connectEventHander( const EventHandler& eventHandler )
{
	mEventHandler = eventHandler;
}

//////////////////////////////////////////////////////////////////////////////////////////////

const char* FaceTracker::Exception::what() const throw() 
{ 
	return mMessage; 
}

FaceTracker::ExcFaceTrackerCreate::ExcFaceTrackerCreate() throw()
{
	sprintf( mMessage, "Unable to create face tracker." );
}

FaceTracker::ExcFaceTrackerCreateImage::ExcFaceTrackerCreateImage( long hr ) throw()
{
	sprintf( mMessage, "Unable to create face tracker image. Error: %i", hr );
}

FaceTracker::ExcFaceTrackerCreateResult::ExcFaceTrackerCreateResult( long hr ) throw()
{
	sprintf( mMessage, "Unable to create face tracker result. Error: %i", hr );
}

FaceTracker::ExcFaceTrackerInit::ExcFaceTrackerInit( long hr ) throw()
{
	sprintf( mMessage, "Unable to initialize face tracker. Error: %i", hr );
}
}
 
#include "FaceTracker.h"
#include "cinder/app/App.h"

using namespace ci;
using namespace KinectSdk;
using namespace std;

FaceTracker::Face::Face()
{
	mMatrix.setToNull();
	mBounds					= Rectf( 0.0f, 0.0f, 0.0f, 0.0f );
	mUserId					= 0;
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

const Matrix44f& FaceTracker::Face::getTransformMatrix() const
{
	return mMatrix;
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
	mImageColor		= 0;
	mImageDepth		= 0;
	mModel			= 0;
	mNewFace		= false;
	mResult			= 0;
	mRunning		= false;
	mSuccess		= false;
}

FaceTracker::~FaceTracker()
{
	stop();

	if ( mFaceTracker != 0 ) {
		mFaceTracker->Release();
		mFaceTracker = 0;
	}

	if ( mImageColor ) {
		mImageColor->Release();
		mImageColor = 0;
	}
	if ( mImageDepth ) {
		mImageDepth->Release();
		mImageDepth = 0;
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

	mImageColor = FTCreateImage();
	if ( !mImageColor || FAILED( hr = mImageColor->Allocate( mConfigColor.Width, mConfigColor.Height, FTIMAGEFORMAT_UINT8_B8G8R8A8 ) ) ) {
		throw ExcFaceTrackerCreateImage( hr );
	}

	mImageDepth = FTCreateImage();
	if ( !mImageDepth || FAILED( hr = mImageDepth->Allocate( mConfigDepth.Width, mConfigDepth.Height, FTIMAGEFORMAT_UINT16_D13P3 ) ) ) {
		throw ExcFaceTrackerCreateImage( hr );
	}

	mRunning	= true;
	//mThread		= ThreadRef( new thread( &FaceTracker::run, this ) );
}

void FaceTracker::stop()
{
	mRunning = false;
	if ( mThread ) {
		mThread->join();
		mThread.reset();
	}
}

void FaceTracker::findFace( const Surface8u& color, const Surface16u& depth, const Vec3f headPoints[ 2 ], size_t userId )
{
	if ( !mNewFace && color && depth ) {
		mHeadPoints.clear();
		if ( headPoints != 0 ) {
			mHeadPoints.push_back( headPoints[ 0 ] );
			mHeadPoints.push_back( headPoints[ 1 ] );
		}
		mUserId		= userId;
		mImageColor->Attach( color.getWidth(), color.getHeight(), (void*)color.getData(), FTIMAGEFORMAT_UINT8_B8G8R8A8, color.getWidth() * 4 );
		mImageDepth->Attach( depth.getWidth(), depth.getHeight(), (void*)depth.getData(), FTIMAGEFORMAT_UINT16_D13P3, depth.getWidth() * 2 );
		run();
	}
}

void FaceTracker::run()
{
	//while ( mRunning ) {
		if ( !mNewFace ) {
			if ( mEventHandler != nullptr ) {
				AnimationUnitMap animationUnitMap;
				Rectf bounds;
				Matrix44f matrix;
				TriMesh mesh;
				TriMesh2d mesh2d;

				long hr = E_FAIL;

				if ( mImageColor != 0 && mImageDepth != 0 ) {
					FT_SENSOR_DATA data;
					tagPOINT offset;
					offset.x			= 0;
					offset.y			= 0;
					data.pDepthFrame	= mImageDepth;
					data.pVideoFrame	= mImageColor;
					data.ViewOffset		= offset;
					data.ZoomFactor		= 1.0f;

					FT_VECTOR3D h0( 0.0f, 0.0f, 0.0f );
					FT_VECTOR3D h1( 0.0f, 0.0f, 0.0f );					
					bool hint = mHeadPoints.size() == 2;
					if ( hint ) {
						h0 = FT_VECTOR3D( mHeadPoints[ 0 ].x, mHeadPoints[ 0 ].y, mHeadPoints[ 0 ].z );
						h1 = FT_VECTOR3D( mHeadPoints[ 1 ].x, mHeadPoints[ 1 ].y, mHeadPoints[ 1 ].z );
					}
					FT_VECTOR3D headPoints[ 2 ] = { h0, h1 };

					if ( mSuccess ) {
						hr = mFaceTracker->ContinueTracking( &data, hint ? headPoints : 0, mResult );
					} else {
						hr = mFaceTracker->StartTracking( &data, 0, hint ? headPoints : 0, mResult );
					}

					if ( SUCCEEDED( hr ) ) {
						hr			= mResult->GetStatus();
						mSuccess	= hr == S_OK;
					}
					
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
									animationUnitMap[ (AnimationUnit)i ] = animationUnits[ i ];
								}
							}

							float scale;
							float rotation[ 3 ];
							float translation[ 3 ];
							hr = mResult->Get3DPose( &scale, rotation, translation );
							if ( SUCCEEDED( hr ) ) {
								Vec3f r( rotation[ 0 ], rotation[ 1 ], rotation[ 2 ] );
								Vec3f t( translation[ 0 ], translation[ 1 ], translation[ 2 ] );

								matrix.setToIdentity();
								matrix.translate( t );
								matrix.rotate( r );
								matrix.translate( -t );
								matrix.translate( t );
								matrix.scale( Vec3f::one() * scale );
							}

							size_t numVertices	= mModel->GetVertexCount();
							
							if ( numAnimationUnits > 0 && numShapeUnits > 0 && numVertices > 0 ) {
								if ( mCalcMesh ) {
									FT_VECTOR3D* pts = reinterpret_cast<FT_VECTOR3D*>( _malloca( sizeof( FT_VECTOR3D ) * numVertices ) );
									hr = mModel->Get3DShape( shapeUnits, numShapeUnits, animationUnits, numAnimationUnits, scale, rotation, translation, pts, numVertices );
									if ( SUCCEEDED( hr ) ) {
										for ( size_t i = 0; i < numVertices; ++i ) {
											Vec3f v( pts[ i ].x, pts[ i ].y, pts[ i ].z );
											mesh.appendVertex( v );
										}

										FT_TRIANGLE* triangles;
										size_t triangleCount;
										hr = mModel->GetTriangles( &triangles, &triangleCount );
										if ( SUCCEEDED( hr ) ) {
											for ( size_t i = 0; i < triangleCount; ++i ) {
												mesh.appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
											}
										}
									}
									_freea( pts );
								}

								if ( mCalcMesh2d ) {
									tagPOINT viewOffset	= { 0, 0 };
									FT_VECTOR2D* pts		= reinterpret_cast<FT_VECTOR2D*>( _malloca( sizeof( FT_VECTOR2D ) * numVertices ) );
									hr = mModel->GetProjectedShape( &mConfigColor, data.ZoomFactor, viewOffset, shapeUnits, numShapeUnits, animationUnits, 
										numAnimationUnits, scale, rotation, translation, pts, numVertices );
									if ( SUCCEEDED( hr ) ) {
										for ( size_t i = 0; i < numVertices; ++i ) {
											Vec2f v( pts[ i ].x + 0.5f, pts[ i ].y + 0.5f );
											mesh2d.appendVertex( v );
										}

										FT_TRIANGLE* triangles;
										size_t triangleCount;
										hr = mModel->GetTriangles( &triangles, &triangleCount );
										if ( SUCCEEDED( hr ) ) {
											for ( size_t i = 0; i < triangleCount; ++i ) {
												mesh2d.appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
											}
										}
									}
									_freea( pts );
								}
							}

							tagRECT rect;
							hr = mResult->GetFaceRect( &rect );
							if ( SUCCEEDED( hr ) ) {
								bounds = Rectf( (float)rect.left, (float)rect.top, (float)rect.right, (float)rect.bottom );
							}

							mModel->Release();
							mModel = 0;
						}
					} else {
						mResult->Reset();
					}
				}

				mFace = Face();
				mFace.mAnimationUnits		= animationUnitMap;
				mFace.mBounds				= bounds;
				mFace.mMatrix				= matrix;
				mFace.mMesh					= mesh;
				mFace.mMesh2d				= mesh2d;
				mFace.mUserId				= mUserId;
				mNewFace = true;
			}
		}
	//}
}

void FaceTracker::update()
{
	if ( mNewFace ) {
		mEventHandler( mFace );
		mNewFace = false;
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
	sprintf( mMessage, "Unable to create face tracker" );
}

FaceTracker::ExcFaceTrackerCreateImage::ExcFaceTrackerCreateImage( long hr ) throw()
{
	sprintf( mMessage, "Unable to create face tracker image: %i", hr );
}

FaceTracker::ExcFaceTrackerCreateResult::ExcFaceTrackerCreateResult( long hr ) throw()
{
	sprintf( mMessage, "Unable to create face tracker result: %i", hr );
}

FaceTracker::ExcFaceTrackerInit::ExcFaceTrackerInit( long hr ) throw()
{
	sprintf( mMessage, "Unable to initialize face tracker: %i", hr );
}
 
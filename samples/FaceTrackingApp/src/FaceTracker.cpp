#include "FaceTracker.h"

using namespace ci;
using namespace KinectSdk;
using namespace std;

FaceTracker::Face::Face()
{
	mMatrix.setToNull();
	mBounds	= Rectf( 0.0f, 0.0f, 0.0f, 0.0f );
	mUserId	= 0;
}

FaceTracker::Face::Face( size_t userId, const Rectf& bounds, const TriMesh2d& mesh, const Matrix44f& matrix )
{
	mBounds	= bounds;
	mMatrix	= matrix;
	mMesh	= mesh;
	mUserId	= userId;
}

const Rectf& FaceTracker::Face::getBounds() const
{
	return mBounds;
}

const TriMesh2d& FaceTracker::Face::getMesh() const
{
	return mMesh;
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
	mEventHandler	= nullptr;
	mFaceTracker	= 0;
	mModel			= 0;
	mNewFace		= false;
	mResult			= 0;
	mRunning		= false;
	mSuccess		= false;
}

FaceTracker::~FaceTracker()
{
	stop();
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

void FaceTracker::findFace( const Surface8u& color, const Surface16u& depth, const Skeleton& skeleton, size_t userId )
{
	if ( !mNewFace ) {
		mSkeleton		= skeleton;
		mSurfaceColor	= color;
		mSurfaceDepth	= depth;
		mUserId			= userId;
	}
}

void FaceTracker::run()
{
	while ( mRunning ) {
		if ( !mNewFace ) {
			if ( mEventHandler != nullptr ) {
				Rectf		bounds;
				Matrix44f	matrix;
				TriMesh2d	mesh;

				long hr = E_FAIL;

				if ( mSurfaceColor && mSurfaceDepth ) {
					IFTImage* colorImage = (IFTImage*)mSurfaceColor.getData();
					IFTImage* depthImage = (IFTImage*)mSurfaceDepth.getChannelRed().getData();
					FT_SENSOR_DATA data( colorImage, depthImage, 1.0f, 0 );

					Vec3f head = Vec3f::zero();
					Vec3f neck = Vec3f::zero();
					if ( mSkeleton.empty() ) {
						head = mSkeleton.at( JointName::NUI_SKELETON_POSITION_HEAD ).getPosition();
						neck = mSkeleton.at( JointName::NUI_SKELETON_POSITION_SHOULDER_CENTER ).getPosition();
					}
					FT_VECTOR3D h( head.x, head.y, head.z );
					FT_VECTOR3D n( neck.x, neck.y, neck.z );
					FT_VECTOR3D hint[ 2 ] = { n, h };
			
					if ( mSuccess ) {
						hr = mFaceTracker->ContinueTracking( &data, hint, mResult );
					} else {
						hr = mFaceTracker->StartTracking( &data, 0, hint, mResult );
					}

					mSuccess = SUCCEEDED( hr ) && SUCCEEDED( mResult->GetStatus() );
					if ( mSuccess ) {
						hr = mFaceTracker->GetFaceModel( &mModel );
						if ( SUCCEEDED( hr ) ) {

							float* pSU			= 0;
							uint32_t numSU		= 0;
							int32_t suConverged	= false;
							mFaceTracker->GetShapeUnits( 0, &pSU, &numSU, &suConverged );
							tagPOINT viewOffset	= { 0, 0 };
					
							size_t vertexCount = mModel->GetVertexCount();
							FT_VECTOR2D* points = reinterpret_cast<FT_VECTOR2D*>( _malloca( sizeof( FT_VECTOR2D ) * vertexCount ) );
							if ( points ) {
								float* aus;
								UINT auCount;
								hr = mResult->GetAUCoefficients( &aus, &auCount );
								if ( SUCCEEDED( hr ) ) {
							
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

										hr = mModel->GetProjectedShape( &mConfigColor, 1.0f, viewOffset, pSU, mModel->GetSUCount(), aus, 
											auCount, scale, rotation, translation, points, vertexCount );
										if ( SUCCEEDED( hr ) ) {
											tagPOINT* model = reinterpret_cast<POINT*>( _malloca( sizeof( POINT ) * vertexCount ) );
											if ( model ) {
												for ( size_t i = 0; i < vertexCount; ++i ) {
													Vec2f v( points[ i ].x + 0.5f, points[ i ].y + 0.5f );
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

												tagRECT rect;
												hr = mResult->GetFaceRect( &rect );
												if ( SUCCEEDED( hr ) ) {
													bounds = Rectf( (float)rect.left, (float)rect.top, (float)rect.right, (float)rect.bottom );
												}

												_freea( model ); 
											}
										}
									}
								}
								_freea( points );
							}
							mModel->Release();
						}
					} else {
						mResult->Reset();
					}
				}

				mFace = Face( mUserId, bounds, mesh, matrix );
				mNewFace = true;
			}
		}
	}
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

FaceTracker::ExcFaceTrackerCreateResult::ExcFaceTrackerCreateResult( long hr ) throw()
{
	sprintf( mMessage, "Unable to create face tracker result: %i", hr );
}

FaceTracker::ExcFaceTrackerInit::ExcFaceTrackerInit( long hr ) throw()
{
	sprintf( mMessage, "Unable to initialize face tracker: %i", hr );
}
 
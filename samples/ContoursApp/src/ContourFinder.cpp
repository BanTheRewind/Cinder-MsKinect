#include "ContourFinder.h"

using namespace ci;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////

Contour::Contour()
{
	mCentroid	= Vec2f::zero();
}

Contour::~Contour()
{
	mPoints.clear();
}

void Contour::addPoint( const Vec2f &position )
{
	mPoints.push_back( position );
}

void Contour::calcCentroid()
{
	mCentroid = Vec2f::zero();
	if ( !mPoints.empty() ) {
		for ( vector<Vec2f>::const_iterator pointIt = mPoints.cbegin(); pointIt != mPoints.cend(); ++pointIt ) {
			mCentroid += *pointIt;
		}
	}
	mCentroid /= (float)mPoints.size();
}

Vec2f& Contour::getCentroid()
{
	return mCentroid;
}

const Vec2f& Contour::getCentroid() const
{
	return mCentroid;
}

vector<Vec2f>& Contour::getPoints()
{
	return mPoints;
}

const vector<Vec2f>& Contour::getPoints() const
{
	return mPoints;
}

///////////////////////////////////////////////////////////////////////////////////////////////

ContourFinderRef ContourFinder::create()
{
	return ContourFinderRef( new ContourFinder() );
}

ContourFinder::ContourFinder()
{
	mChannelBackground = Channel8u( 80, 60 );
	Channel8u::Iter iter = mChannelBackground.getIter();
	while ( iter.line() ) {
		while ( iter.pixel() ) {
			mChannelBackground.setValue( iter.getPos(), 255 );
		}
	}
	mCvMatBackground = toOcv( mChannelBackground );
}

ContourFinder::~ContourFinder()
{
	mCvMatBackground.release();
	mCvMatDiff.release();
	mCvMatFrame.release();
	mCvMatTemp.release();
}

vector<Contour> ContourFinder::findContours( const Channel8u &channel, int32_t smoothing )
{
	vector<Contour> contours;
	vector<vector<cv::Point> > cvContours;

	// Convert image to OpenCV
	mCvMatFrame = toOcv( channel );

	// THreshold the image
	cv::threshold( mCvMatFrame, mCvMatFrame, 0.5, 255.0, CV_THRESH_BINARY );
	
	// Find difference between input channel and solid background
	cv::absdiff( mCvMatBackground, mCvMatFrame, mCvMatDiff );

	// Do not evaluate a pure-zero image or an exception will occur
	if ( cv::countNonZero( mCvMatDiff ) > 0 ) {

		// Find contours
		cv::findContours( mCvMatDiff, cvContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
		for ( vector<vector<cv::Point> >::const_iterator contourIt = cvContours.cbegin(); contourIt != cvContours.cend(); ++contourIt ) {
			Contour contour;
			
			// Iterate through points in CV contour. Contour smoothing skips over 
			// points to simplify the outline for faster tracking.
			int32_t i = 0;
			for ( vector<cv::Point>::const_iterator pointIt = contourIt->cbegin(); pointIt != contourIt->cend(); ++pointIt, i++ ) {
				if ( smoothing == 0 || i % smoothing == 0 ) {
					Vec2f point( (float)pointIt->x, (float)pointIt->y );
					contour.addPoint( point );	
				}
			}
			
			// Add contour
			contours.push_back( contour );
							
		}

	}

	return contours;
}

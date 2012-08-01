#pragma once

#include "CinderOpenCV.h"

///////////////////////////////////////////////////////////////////////////////////////////////

class ContourFinder;

class Contour
{
public:
	Contour();
	~Contour();

	ci::Vec2f&						getCentroid();
	const ci::Vec2f&				getCentroid() const;
	std::vector<ci::Vec2f>&			getPoints();
	const std::vector<ci::Vec2f>&	getPoints() const;

	void							calcCentroid();
private:
	void							addPoint( const ci::Vec2f &position );

	ci::Vec2f						mCentroid;
	std::vector<ci::Vec2f>			mPoints;

	friend class					ContourFinder;	
};

///////////////////////////////////////////////////////////////////////////////////////////////

typedef std::shared_ptr<ContourFinder> ContourFinderRef;

class ContourFinder
{
public:
	static ContourFinderRef	create();
	~ContourFinder();

	std::vector<Contour>	findContours( const ci::Channel8u &channel, int32_t smoothing = 0 );
private:
	ContourFinder();
	
	ci::Channel8u			mChannelBackground;
	cv::Mat					mCvMatBackground;
	cv::Mat					mCvMatDiff;
	cv::Mat					mCvMatFrame;
	cv::Mat					mCvMatTemp;
};

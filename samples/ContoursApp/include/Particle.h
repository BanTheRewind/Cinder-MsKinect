#pragma once

#include "cinder/Color.h"
#include "cinder/Vector.h"

class Particle 
{
public:
	Particle( const ci::Vec2f& position = ci::Vec2f::zero(), const ci::Colorf &color = ci::ColorAf::white() );

	void				addAcceleration( const ci::Vec2f &acceleration );
	
	const ci::Colorf&	getColor() const;
	const ci::Vec2f&	getPosition() const;

	void				update();
private:
	ci::Vec2f			mAcceleration;
	ci::Colorf			mColor;
	ci::Vec2f			mOrigin;
	ci::Vec2f			mPosition;
	ci::Vec2f			mVelocity;
};

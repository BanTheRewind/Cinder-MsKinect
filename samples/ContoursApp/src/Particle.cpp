#include "Particle.h"

#include "cinder/Rand.h"

using namespace ci;

/////////////////////////////////////////////////////////////////////////////

Particle::Particle( const Vec2f &position, const Colorf &color ) 
{
	mAcceleration		= Vec2f::zero();
	mColor				= color;
	mOrigin				= position;
	mPosition			= mOrigin;
	mVelocity			= Vec2f::zero();
}

void Particle::addAcceleration( const Vec2f &acceleration )
{ 
	mAcceleration += acceleration;
}

const Colorf& Particle::getColor() const 
{ 
	return mColor; 
}

const Vec2f& Particle::getPosition() const 
{ 
	return mPosition; 
}

void Particle::update()
{
	mVelocity		+= mAcceleration;
	mPosition		+= mVelocity * 0.333f;
	mVelocity		*= 0.96f;
	mAcceleration	= Vec2f::zero();

	float distance	= mOrigin.distance( mPosition );
	if ( distance < 0.1f ) {
		mPosition	= mOrigin;
		mVelocity	= Vec2f::zero();
	} else {
		mPosition	+= ( mOrigin - mPosition ) * 0.21f;
	}
}

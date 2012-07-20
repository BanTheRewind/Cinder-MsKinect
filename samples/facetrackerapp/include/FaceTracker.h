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

#pragma once

#include "cinder/Vector.h"
#include "cinder/Rect.h"
#include "cinder/Surface.h"
#include <FaceTrackLib.h>
#include "Kinect.h"

typedef std::shared_ptr<class FaceTracker>	FaceTrackerRef;

class FaceTracker
{
public:
	static FaceTrackerRef			create();
	~FaceTracker();

	bool							findFace( const KinectSdk::Skeleton& skeleton, const ci::Surface8u &videoSurface, 
		const ci::Surface16u &depthSurface );
	const ci::Rectf&				getBounds() const;
	const ci::Vec2f&				getCenter() const;
	const std::vector<ci::Vec2f>&	getFace() const;
	void							setup( const KinectSdk::DeviceOptions &deviceOptions );
private:

	FaceTracker();

	bool							mTracking;

	ci::Rectf						mBounds;
	ci::Vec2f						mCenter;
	FT_CAMERA_CONFIG*				mConfigDepth;
	FT_CAMERA_CONFIG*				mConfigVideo;
	IFTImage						*mDepthImage;
	std::vector<ci::Vec2f>			mFace;
	IFTFaceTracker					*mFaceTracker;
	IFTResult						*mFaceTrackerResult;
	FT_VECTOR3D						mHint[ 2 ];
	bool							mLastTrackSucceeded;
	IFTImage						*mVideoImage;
	void							setCenter();
};

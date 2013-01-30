/*
 * This file is part of ALVAR, A Library for Virtual and Augmented Reality.
 *
 * Copyright 2007-2012 VTT Technical Research Centre of Finland
 *
 * Contact: VTT Augmented Reality Team <alvar.info@vtt.fi>
 *          <http://www.vtt.fi/multimedia/alvar.html>
 *
 * ALVAR is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ALVAR; if not, see
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>.
 */

#ifndef TRACKERORIENTATION_H
#define TRACKERORIENTATION_H

#include "Tracker.h"
#include "TrackerFeatures.h"
#include "Pose.h"
#include "Camera.h"

/**
 * \file TrackerOrientation.h
 *
 * \brief This file implements an orientation tracker.
 */

namespace alvar {

/**
 * \brief Track orientation based only on features in the image plane.
 */
class ALVAR_EXPORT TrackerOrientation : public Tracker {
	
public:
	TrackerOrientation(int width, int height, int image_scale=1, int outlier_limit=20)
		: _image_scale(image_scale)
		, _outlier_limit(outlier_limit)
		, _xres(width)
		, _yres(height)
	{
		_camera		  = 0;
		_grsc		  = 0;
		_object_model = 0;
	}

private:

	struct Feature
	{
		enum {NOT_TRACKED=0, IS_TRACKED} status2D;
		enum {NONE=0, USE_FOR_POSE, IS_OUTLIER, IS_INITIAL} status3D;

		Feature()
		{
			status3D = NONE;
			status2D = NOT_TRACKED;
		}

		Feature(double vx, double vy)
		{
			point.x   = float(vx);
			point.y   = float(vy);
			status3D  = NONE;
			status2D  = NOT_TRACKED;
		}
	 
		~Feature() {}

		CvPoint2D32f point;
		CvPoint3D64f point3d;
	};

	TrackerFeatures		  _ft;
	std::map<int,Feature> _F_v;

	int					  _xres;
	int					  _yres;
	int					  _image_scale;
	int					  _outlier_limit;
	
	Pose				  _pose;
	IplImage			 *_grsc;
	Camera				 *_camera;
	CvMat				 *_object_model;

public:
	void SetCamera(Camera *camera) {
		_camera = camera;
	}
	void GetPose(Pose &pose);
	void GetPose(double gl_mat[16])	{
		_pose.GetMatrixGL(gl_mat);
	}
	void Reset();
	double Track(IplImage *image);

private:
	static void Project(CvMat* state, CvMat* projection, void *param);
	bool UpdatePose(IplImage* image=0);
	bool UpdateRotationOnly(IplImage *gray, IplImage *image=0);

};

} // namespace alvar

#endif



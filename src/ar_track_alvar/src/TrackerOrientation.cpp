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

#include "TrackerOrientation.h"
#include "Optimization.h"

using namespace std;

namespace alvar {
using namespace std;

void TrackerOrientation::Project(CvMat* state, CvMat* projection, void *param)
{
	TrackerOrientation *tracker = (TrackerOrientation*)param;
	int count = projection->rows;
	CvMat rot_mat = cvMat(3, 1, CV_64F, &(state->data.db[0+0]));
	double zeros[3] = {0};
	CvMat zero_tra = cvMat(3, 1, CV_64F, zeros);
	cvReshape(projection, projection, 2, 1);
	cvProjectPoints2(tracker->_object_model, &rot_mat, &zero_tra, &(tracker->_camera->calib_K), &(tracker->_camera->calib_D), projection);
	cvReshape(projection, projection, 1, count);
}

void TrackerOrientation::Reset()
{	
	_pose.Reset();
	_pose.Mirror(false, true, true);
}

// Pose difference is stored...
double TrackerOrientation::Track(IplImage *image)
{
	UpdateRotationOnly(image, 0);
	return 0;	
}

bool TrackerOrientation::UpdatePose(IplImage *image)
{
	int count_points = _F_v.size();
	if(count_points < 6) return false;

	CvMat* _M = cvCreateMat(count_points, 1, CV_64FC3);
	CvMat* image_observations = cvCreateMat(count_points*2, 1, CV_64F); // [u v u v u v ...]'

	//map<int,Feature>::iterator it;
	int ind = 0;
	for(map<int,Feature>::iterator it=_F_v.begin(); it!=_F_v.end(); it++)	
	{
		if((it->second.status3D  == Feature::USE_FOR_POSE || 
			it->second.status3D  == Feature::IS_INITIAL)  && 
			it->second.status2D  == Feature::IS_TRACKED)
		{
			_M->data.db[ind*3+0] = it->second.point3d.x;
			_M->data.db[ind*3+1] = it->second.point3d.y;
			_M->data.db[ind*3+2] = it->second.point3d.z;

			image_observations->data.db[ind*2+0] = it->second.point.x;
			image_observations->data.db[ind*2+1] = it->second.point.y;
			ind++;
		}
	}

	if(ind < 6)
	{			
		cvReleaseMat(&image_observations);
		cvReleaseMat(&_M);
		return false;		
	}

	double rot[3]; CvMat rotm = cvMat(3, 1, CV_64F, rot);
	_pose.GetRodriques(&rotm);

	CvMat* par = cvCreateMat(3, 1, CV_64F);
	memcpy(&(par->data.db[0+0]), rot, 3*sizeof(double));
	//par->data.db[3] = 0;

	CvRect r; r.x = 0; r.y = 0; r.height = ind; r.width = 1;
	CvMat Msub;
	cvGetSubRect(_M, &Msub, r);
	_object_model = &Msub;

	r.height = 2*ind;
	CvMat image_observations_sub;
	cvGetSubRect(image_observations, &image_observations_sub, r);

	alvar::Optimization *opt = new alvar::Optimization(3, 2*ind);
	
	double foo = opt->Optimize(par, &image_observations_sub, 0.0005, 5, Project, this, alvar::Optimization::TUKEY_LM);
	memcpy(rot, &(par->data.db[0+0]), 3*sizeof(double));
	_pose.SetRodriques(&rotm);
	
	delete opt;

	cvReleaseMat(&par);
	cvReleaseMat(&image_observations);
	cvReleaseMat(&_M);

	return true;
}

bool TrackerOrientation::UpdateRotationOnly(IplImage *gray, IplImage *image)
{
	if(gray->nChannels != 1) return false;
	if(!_grsc) _grsc = cvCreateImage(cvSize(_xres, _yres), 8, 1);
	if((_xres!=_grsc->width)||(_yres!=_grsc->height))
		cvResize(gray, _grsc);
	else
		_grsc->imageData =  gray->imageData;

	map<int,Feature>::iterator it ;
	for(it = _F_v.begin(); it != _F_v.end(); it++)
		it->second.status2D = Feature::NOT_TRACKED;

	// Track features in image domain (distorted points)
	_ft.Track(_grsc);

	// Go through image features and match to previous (_F_v)
	for (int i = 0; i < _ft.feature_count; i++)
	{
		int id            = _ft.ids[i];
		_F_v[id].point    = _ft.features[i];
		_F_v[id].point.x *= _image_scale;
		_F_v[id].point.y *= _image_scale;
		_F_v[id].status2D = Feature::IS_TRACKED;
		
		// Throw outlier away
		if(_F_v[id].status3D == Feature::IS_OUTLIER)
		{
			_ft.DelFeature(i);
		}			
	}

	// Delete features that are not tracked
//		map<int,Feature>::iterator 
	it = _F_v.begin();
	while(it != _F_v.end())
	{
		if(it->second.status2D == Feature::NOT_TRACKED)
			_F_v.erase(it++);
		else ++it;
	}
	
	// Update pose based on current information
	UpdatePose();

	it = _F_v.begin();
	while(it != _F_v.end())
	{
		Feature *f = &(it->second);

		// Add new points
		if(f->status3D == Feature::NONE)
		{
			double wx, wy, wz;
			
			CvPoint2D32f fpu = f->point;
			_camera->Undistort(fpu);
			
			// Tassa asetetaan z = inf ja lasketaan x ja y jotenkin?!?
			int object_scale = 1; // TODO Same as the pose?!?!?!?
							
			// inv(K)*[u v 1]'*scale
			wx =  object_scale*(fpu.x-_camera->calib_K_data[0][2])/_camera->calib_K_data[0][0];
			wy =  object_scale*(fpu.y-_camera->calib_K_data[1][2])/_camera->calib_K_data[1][1];
			wz =  object_scale;

			// Now the points are in camera coordinate frame.
			alvar::Pose p = _pose;
			p.Invert();
			
			double Xd[4] = {wx, wy, wz, 1};
			CvMat Xdm = cvMat(4, 1, CV_64F, Xd);
			double Pd[16];
			CvMat Pdm = cvMat(4, 4, CV_64F, Pd);
			p.GetMatrix(&Pdm);
			cvMatMul(&Pdm, &Xdm, &Xdm);
			f->point3d.x = Xd[0]/Xd[3];
			f->point3d.y = Xd[1]/Xd[3];
			f->point3d.z = Xd[2]/Xd[3];
			//cout<<f->point3d.z<<endl;

			f->status3D = Feature::USE_FOR_POSE;
		}

		if(image)
		{
			if(f->status3D == Feature::NONE)
				cvCircle(image, cvPoint(int(f->point.x), int(f->point.y)), 3, CV_RGB(255,0,0), 1);
			else if(f->status3D == Feature::USE_FOR_POSE)
				cvCircle(image, cvPoint(int(f->point.x), int(f->point.y)), 3, CV_RGB(0,255,0), 1);
			else if(f->status3D == Feature::IS_INITIAL)
				cvCircle(image, cvPoint(int(f->point.x), int(f->point.y)), 3, CV_RGB(0,0,255), 1);
			else if(f->status3D == Feature::IS_OUTLIER)
				cvCircle(image, cvPoint(int(f->point.x), int(f->point.y)), 2, CV_RGB(255,0,255), 1);
		}

		// Delete points that bk error is too big
		// OK here just change state...
		// NYT TEHAAN TURHAAN ASKEN MUKAAN OTETUILLE..
		if( f->status3D == Feature::USE_FOR_POSE || 
			f->status3D == Feature::IS_INITIAL )
		{				
			double p3d[3] = {f->point3d.x, f->point3d.y, f->point3d.z};
			CvMat p3dm = cvMat(1, 1, CV_64FC3, p3d);
			double p2d[2];
			CvMat p2dm = cvMat(2, 1, CV_64F, p2d);
			cvReshape(&p2dm, &p2dm, 2, 1);
			
			double gl_mat[16];
			_pose.GetMatrixGL(gl_mat);
			_camera->ProjectPoints(&p3dm, gl_mat, &p2dm);

			if(image)
				cvLine(image, cvPoint(int(p2d[0]), int(p2d[1])), cvPoint(int(f->point.x),int(f->point.y)), CV_RGB(255,0,255));

			double dist = (p2d[0]-f->point.x)*(p2d[0]-f->point.x)+(p2d[1]-f->point.y)*(p2d[1]-f->point.y);
			if(dist > _outlier_limit)
				f->status3D = Feature::IS_OUTLIER;
		}

		it++;
	}
	return true;
}

} // namespace alvar

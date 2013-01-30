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

#include "EC.h"
#include "Optimization.h"

namespace alvar {

struct ProjectParams
{
	Camera *camera;
	const CvMat *object_model;
};

static void ProjectRot(CvMat* state, CvMat* projection, void* x) 
{
	ProjectParams *foo = (ProjectParams*)x;
	Camera *camera = foo->camera;
	const CvMat *object_model = foo->object_model;
	int count = projection->rows;
	CvMat rot_mat = cvMat(3, 1, CV_64F, &(state->data.db[0+0]));
	double zeros[3] = {0};
	CvMat zero_tra = cvMat(3, 1, CV_64F, zeros);
	cvReshape(projection, projection, 2, 1);
	cvProjectPoints2(object_model, &rot_mat, &zero_tra, &(camera->calib_K), &(camera->calib_D), projection);
	cvReshape(projection, projection, 1, count);
}

// TODO: How this differs from the Camera::ProjectPoints ???
static void Project(CvMat* state, CvMat* projection, void* x)
{
	ProjectParams *foo = (ProjectParams*)x;
	Camera *camera = foo->camera;
	const CvMat *object_model = foo->object_model;
	int count = projection->rows;
	CvMat rot_mat = cvMat(3, 1, CV_64F, &(state->data.db[0+0]));
	CvMat tra_mat = cvMat(3, 1, CV_64F, &(state->data.db[0+3]));
	cvReshape(projection, projection, 2, 1);
	cvProjectPoints2(object_model, &rot_mat, &tra_mat, &(camera->calib_K), &(camera->calib_D), projection);
	cvReshape(projection, projection, 1, count);
}

bool CameraEC::UpdatePose(const CvMat* object_points, CvMat* image_points, Pose *pose, CvMat *weights) {
	double rot[3]; CvMat rotm = cvMat(3, 1, CV_64F, rot);
	double tra[3]; CvMat tram = cvMat(3, 1, CV_64F, tra);
	pose->GetRodriques(&rotm);
	pose->GetTranslation(&tram);
	bool ret = UpdatePose(object_points, image_points, &rotm, &tram, weights);
	pose->SetRodriques(&rotm);
	pose->SetTranslation(&tram);
	return ret;
}

bool CameraEC::UpdatePose(const CvMat* object_points, CvMat* image_points, CvMat *rot, CvMat *tra, CvMat *weights) {
	if (object_points->height < 4) return false;
	/*	if (object_points->height < 6) {
		return false;
		// TODO: We need to change image_points into CV_32FC2
		return Camera::CalcExteriorOrientation(object_points, image_points, rot, tra);
	}*/
	CvMat* par = cvCreateMat(6, 1, CV_64F);
	memcpy(&(par->data.db[0+0]), rot->data.db, 3*sizeof(double));
	memcpy(&(par->data.db[0+3]), tra->data.db, 3*sizeof(double));

	ProjectParams pparams;
	pparams.camera = this;
	pparams.object_model  = object_points;

	alvar::Optimization *opt = new alvar::Optimization(6, image_points->height);
	double tmp = opt->Optimize(par, image_points, 0.0005, 2, Project, &pparams, alvar::Optimization::TUKEY_LM, 0, 0, weights);

	memcpy(rot->data.db, &(par->data.db[0+0]), 3*sizeof(double));
	memcpy(tra->data.db, &(par->data.db[0+3]), 3*sizeof(double));
	
	delete opt;

	cvReleaseMat(&par);
	return true;
}

bool CameraEC::UpdateRotation(const CvMat* object_points, CvMat* image_points, Pose *pose)
{
	double rot[3]; CvMat rotm = cvMat(3, 1, CV_64F, rot);
	double tra[3]; CvMat tram = cvMat(3, 1, CV_64F, tra);
	pose->GetRodriques(&rotm);
	pose->GetTranslation(&tram);
	bool ret = UpdateRotation(object_points, image_points, &rotm, &tram);
	pose->SetRodriques(&rotm);
	pose->SetTranslation(&tram);
	return ret;	
}

bool CameraEC::UpdateRotation(const CvMat* object_points, CvMat* image_points, CvMat *rot, CvMat *tra) {

	CvMat* par = cvCreateMat(3, 1, CV_64F);
	memcpy(&(par->data.db[0+0]), rot->data.db, 3*sizeof(double));
	ProjectParams pparams;
	pparams.camera = this;
	pparams.object_model  = object_points;
	alvar::Optimization *opt = new alvar::Optimization(3, image_points->height);
	double tmp = opt->Optimize(par, image_points, 0.0005, 2, ProjectRot, &pparams, alvar::Optimization::TUKEY_LM);
	memcpy(rot->data.db, &(par->data.db[0+0]), 3*sizeof(double));
	delete opt;
	cvReleaseMat(&par);
	return true;
}

// Ol etta mirror asia on kunnossa
void GetOrigo(Pose* pose, CvMat* O)
{
	pose->GetTranslation(O);
}

void GetPointOnLine(const Pose* pose, Camera *camera, const CvPoint2D32f *u, CvMat* P)
{
	double kid[9], rotd[9], trad[3], ud[3] = {u->x, u->y, 1};
	CvMat Ki = cvMat(3, 3, CV_64F, kid);
	CvMat R = cvMat(3, 3, CV_64F, rotd);
	CvMat T = cvMat(3, 1, CV_64F, trad);
	CvMat U = cvMat(3, 1, CV_64F, ud);
	pose->GetMatrix(&R);
	pose->GetTranslation(&T);
	cvInv(&(camera->calib_K), &Ki);
	cvMatMul(&R, &Ki, &Ki);
	cvGEMM(&Ki, &U, 1, &T, 1, P, 0);					
}

bool MidPointAlgorithm(CvMat* o1, CvMat* o2, CvMat* p1, CvMat* p2, CvPoint3D32f& X, double limit)
{
	double ud[3], vd[3], wd[3];
	CvMat u = cvMat(3, 1, CV_64F, ud);
	CvMat v = cvMat(3, 1, CV_64F, vd);
	CvMat w = cvMat(3, 1, CV_64F, wd);
	
	cvSub(p1, o1, &u);
	cvSub(p2, o2, &v);
	cvSub(o1, o2, &w);

	double a = cvDotProduct(&u, &u);
	double b = cvDotProduct(&u, &v);
	double c = cvDotProduct(&v, &v);
	double d = cvDotProduct(&u, &w);
	double e = cvDotProduct(&v, &w);
	double D = a*c - b*b;
	double sc, tc;

	// compute the line parameters of the two closest points
	if (D < limit)
	{   
		return false;
		// the lines are almost parallel
		sc = 0.0;
		tc = (b>c ? d/b : e/c);   // use the largest denominator
	}
	else
	{
		sc = (b*e - c*d) / D;
		tc = (a*e - b*d) / D;
	}
	
	double m1d[3], m2d[3];
	CvMat m1 = cvMat(3, 1, CV_64F, m1d);
	CvMat m2 = cvMat(3, 1, CV_64F, m2d);
	cvAddWeighted(&u, sc, o1, 1.0, 0.0, &m1);
	cvAddWeighted(&v, tc, o2, 1.0, 0.0, &m2);
	cvAddWeighted(&m1, 0.5, &m2, 0.5, 0.0, &m1);	
	
	X.x = (float)m1d[0];
	X.y = (float)m1d[1];
	X.z = (float)m1d[2];

	return true;
}

// todo
bool CameraEC::ReconstructFeature(const Pose *pose1, const Pose *pose2, const CvPoint2D32f *u1, const CvPoint2D32f *u2, CvPoint3D32f *p3d, double limit) {
	double o1d[3], o2d[3], p1d[3], p2d[3];
	CvMat o1 = cvMat(3, 1, CV_64F, o1d);
	CvMat o2 = cvMat(3, 1, CV_64F, o2d);
	CvMat p1 = cvMat(3, 1, CV_64F, p1d);
	CvMat p2 = cvMat(3, 1, CV_64F, p2d);

	Pose po1 = *pose1; // Make copy so that we don't destroy the pose content
	Pose po2 = *pose2;
	po1.Invert();
	po2.Invert();
	GetOrigo(&po1, &o1);
	GetOrigo(&po2, &o2);
	GetPointOnLine(&po1, this, u1, &p1);
	GetPointOnLine(&po2, this, u2, &p2);
	
	return MidPointAlgorithm(&o1, &o2, &p1, &p2, *p3d, limit);
}

void CameraEC::Get3dOnPlane(const Pose *pose, CvPoint2D32f p2d, CvPoint3D32f &p3d) {
	double pd[16], md[9], kd[9];	
	CvMat P = cvMat(4, 4, CV_64F, pd);
	CvMat H = cvMat(3, 3, CV_64F, md);
	CvMat Ki = cvMat(3, 3, CV_64F, kd);
	
	pose->GetMatrix(&P);
	cvInv(&(calib_K), &Ki);
	
	// Construct homography from pose
	int ind_s = 0, ind_c = 0;
	for(int i = 0; i < 3; ++i)
	{
		CvRect r; r.x = ind_s; r.y = 0; r.height = 3; r.width = 1;
		CvMat sub = cvMat(3, 1, CV_64F);
		cvGetSubRect(&P, &sub, r);
		CvMat col = cvMat(3, 1, CV_64F);
		cvGetCol(&H, &col, ind_c);
		cvCopy(&sub, &col);
		ind_c++;
		ind_s++;
		if(i == 1) ind_s++; 
	}

	// Apply H to get the 3D coordinates
	Camera::Undistort(p2d);
	double xd[3] = {p2d.x, p2d.y, 1};
	CvMat X = cvMat(3, 1, CV_64F, xd);
	cvMatMul(&Ki, &X, &X);
	cvInv(&H, &H);
	cvMatMul(&H, &X, &X);

	p3d.x = (float)(xd[0] / xd[2]);
	p3d.y = (float)(xd[1] / xd[2]);
	p3d.z = 0;
}

void CameraEC::Get3dOnDepth(const Pose *pose, CvPoint2D32f p2d, float depth, CvPoint3D32f &p3d)
{
	double wx, wy, wz;
	Camera::Undistort(p2d);
	
	// Tassa asetetaan z = inf ja lasketaan x ja y jotenkin?!?
	//double object_scale = _dist_to_target; // TODO Same as the pose?!?!?!?
					
	// inv(K)*[u v 1]'*scale
	wx =  depth*(p2d.x-calib_K_data[0][2])/calib_K_data[0][0];
	wy =  depth*(p2d.y-calib_K_data[1][2])/calib_K_data[1][1];
	wz =  depth;

	// Now the points are in camera coordinate frame.
	alvar::Pose p = *pose;
	p.Invert();
	
	double Xd[4] = {wx, wy, wz, 1};
	CvMat Xdm = cvMat(4, 1, CV_64F, Xd);
	double Pd[16];
	CvMat Pdm = cvMat(4, 4, CV_64F, Pd);
	p.GetMatrix(&Pdm);
	cvMatMul(&Pdm, &Xdm, &Xdm);
	p3d.x = float(Xd[0]/Xd[3]);
	p3d.y = float(Xd[1]/Xd[3]);
	p3d.z = float(Xd[2]/Xd[3]);
}

} // namespace alvar


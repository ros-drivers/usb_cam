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

#include "Alvar.h"
#include "Camera.h"
#include "FileFormatUtils.h"
#include <memory>

using namespace std;

namespace alvar {
using namespace std;


void ProjPoints::Reset() {
	object_points.clear();
	image_points.clear();
	point_counts.clear();
}

// TODO: Does it matter what the etalon_square_size is???
bool ProjPoints::AddPointsUsingChessboard(IplImage *image, double etalon_square_size, int etalon_rows, int etalon_columns, bool visualize) {
	if (image->width == 0) return false;
	IplImage *gray = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
	CvPoint2D32f *corners = new CvPoint2D32f[etalon_rows*etalon_columns];
	if (image->nChannels == 1) 
		cvCopy(image, gray);
	else
		cvCvtColor(image, gray, CV_RGB2GRAY);
	width = image->width;
	height = image->height;

	int point_count = 0;
	int pattern_was_found = cvFindChessboardCorners(gray, cvSize(etalon_rows, etalon_columns), corners, &point_count);
	if (!pattern_was_found) point_count=0;
	if (point_count > 0) {
		cvFindCornerSubPix(gray, corners, point_count, cvSize(5,5), cvSize(-1,-1),
			cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.01f));
		for (int i=0; i<point_count; i++) {
			CvPoint3D64f po;
			CvPoint2D64f pi;
			po.x = etalon_square_size*(i%etalon_rows);
			po.y = etalon_square_size*(i/etalon_rows);
			po.z = 0;
			pi.x = corners[i].x;
			pi.y = corners[i].y;
			object_points.push_back(po);
			image_points.push_back(pi);
		}
		point_counts.push_back(point_count);
	}
	if (visualize) {
		cvDrawChessboardCorners(image, cvSize(etalon_rows, etalon_columns),
                      corners, point_count, false /*pattern_was_found*/);
	}
	delete [] corners;
	cvReleaseImage(&gray);
	if (point_count > 0) return true;
	return false;
}

bool ProjPoints::AddPointsUsingMarkers(vector<PointDouble> &marker_corners, vector<PointDouble> &marker_corners_img, IplImage* image)
{
	width = image->width;
	height = image->height;
	if ((marker_corners.size() == marker_corners_img.size()) &&
		(marker_corners.size() == 4))
	{
		for (size_t p=0; p<marker_corners.size(); p++) {
			CvPoint3D64f po;
			CvPoint2D64f pi;
			po.x = marker_corners[p].x;
			po.y = marker_corners[p].y;
			po.z = 0;
			pi.x = marker_corners_img[p].x;
			pi.y = marker_corners_img[p].y;
			object_points.push_back(po);
			image_points.push_back(pi);
		}
		point_counts.push_back(marker_corners.size());
	}

	return true;
}

Camera::Camera() {
	calib_K = cvMat(3, 3, CV_64F, calib_K_data);
	calib_D = cvMat(4, 1, CV_64F, calib_D_data);
	memset(calib_K_data,0,sizeof(double)*3*3);
	memset(calib_D_data,0,sizeof(double)*4);
	calib_K_data[0][0] = 550; // Just some focal length by default
	calib_K_data[1][1] = 550; // Just some focal length by default
	calib_K_data[0][2] = 320;
	calib_K_data[1][2] = 240;
	calib_K_data[2][2] = 1;
	calib_x_res = 640;
	calib_y_res = 480;
	x_res = 640;
	y_res = 480;
}


Camera::Camera(ros::NodeHandle & n, std::string cam_info_topic):n_(n) 
{
	calib_K = cvMat(3, 3, CV_64F, calib_K_data);
	calib_D = cvMat(4, 1, CV_64F, calib_D_data);
	memset(calib_K_data,0,sizeof(double)*3*3);
	memset(calib_D_data,0,sizeof(double)*4);
	calib_K_data[0][0] = 550; // Just some focal length by default
	calib_K_data[1][1] = 550; // Just some focal length by default
	calib_K_data[0][2] = 320;
	calib_K_data[1][2] = 240;
	calib_K_data[2][2] = 1;
	calib_x_res = 640;
	calib_y_res = 480;
	x_res = 640;
	y_res = 480;
	cameraInfoTopic_ = cam_info_topic;
	ROS_INFO ("Subscribing to info topic");
    sub_ = n_.subscribe (cameraInfoTopic_, 1, &Camera::camInfoCallback, this);
    getCamInfo_ = false;
}



//
//Camera::Camera(int w, int h) {
//	calib_K = cvMat(3, 3, CV_64F, calib_K_data);
//	calib_D = cvMat(4, 1, CV_64F, calib_D_data);
//	memset(calib_K_data,0,sizeof(double)*3*3);
//	memset(calib_D_data,0,sizeof(double)*4);
//	calib_K_data[0][0] = w/2;
//	calib_K_data[1][1] = w/2;
//	calib_K_data[0][2] = w/2;
//	calib_K_data[1][2] = h/2;
//	calib_K_data[2][2] = 1;
//	calib_x_res = w;
//	calib_y_res = h;
//	x_res = w;
//	y_res = h;
//}

void Camera::SetSimpleCalib(int _x_res, int _y_res, double f_fac)
{
	memset(calib_K_data,0,sizeof(double)*3*3);
	memset(calib_D_data,0,sizeof(double)*4);
	calib_K_data[0][0] = _x_res*f_fac; // Just some focal length by default
	calib_K_data[1][1] = _x_res*f_fac; // Just some focal length by default
	calib_K_data[0][2] = _x_res/2;
	calib_K_data[1][2] = _y_res/2;
	calib_K_data[2][2] = 1;
	calib_x_res = _x_res;
	calib_y_res = _y_res;
}

bool Camera::LoadCalibXML(const char *calibfile) {
	TiXmlDocument document;
	if (!document.LoadFile(calibfile)) return false;
	TiXmlElement *xml_root = document.RootElement();

	return 
		xml_root->QueryIntAttribute("width", &calib_x_res) == TIXML_SUCCESS &&
		xml_root->QueryIntAttribute("height", &calib_y_res) == TIXML_SUCCESS &&
		FileFormatUtils::parseXMLMatrix(xml_root->FirstChildElement("intrinsic_matrix"), &calib_K) && 
		FileFormatUtils::parseXMLMatrix(xml_root->FirstChildElement("distortion"), &calib_D);
}

bool Camera::LoadCalibOpenCV(const char *calibfile) {
	cvSetErrMode(CV_ErrModeSilent);
	CvFileStorage* fs = cvOpenFileStorage(calibfile, 0, CV_STORAGE_READ); 
	cvSetErrMode(CV_ErrModeLeaf);
	if(fs){
		CvFileNode* root_node = cvGetRootFileNode(fs);
		// K Intrinsic
		CvFileNode* intrinsic_mat_node = cvGetFileNodeByName(fs, root_node, "intrinsic_matrix");
		CvMat* intrinsic_mat = reinterpret_cast<CvMat*>(cvRead(fs, intrinsic_mat_node));
		cvmSet(&calib_K, 0, 0, cvmGet(intrinsic_mat, 0, 0));
		cvmSet(&calib_K, 0, 1, cvmGet(intrinsic_mat, 0, 1));
		cvmSet(&calib_K, 0, 2, cvmGet(intrinsic_mat, 0, 2));
		cvmSet(&calib_K, 1, 0, cvmGet(intrinsic_mat, 1, 0));
		cvmSet(&calib_K, 1, 1, cvmGet(intrinsic_mat, 1, 1));
		cvmSet(&calib_K, 1, 2, cvmGet(intrinsic_mat, 1, 2));
		cvmSet(&calib_K, 2, 0, cvmGet(intrinsic_mat, 2, 0));
		cvmSet(&calib_K, 2, 1, cvmGet(intrinsic_mat, 2, 1));
		cvmSet(&calib_K, 2, 2, cvmGet(intrinsic_mat, 2, 2));

		// D Distortion
		CvFileNode* dist_mat_node = cvGetFileNodeByName(fs, root_node, "distortion");
		CvMat* dist_mat = reinterpret_cast<CvMat*>(cvRead(fs, dist_mat_node));
		cvmSet(&calib_D, 0, 0, cvmGet(dist_mat, 0, 0));
		cvmSet(&calib_D, 1, 0, cvmGet(dist_mat, 1, 0));
		cvmSet(&calib_D, 2, 0, cvmGet(dist_mat, 2, 0));
		cvmSet(&calib_D, 3, 0, cvmGet(dist_mat, 3, 0));

		// Resolution
		CvFileNode* width_node = cvGetFileNodeByName(fs, root_node, "width");
		CvFileNode* height_node = cvGetFileNodeByName(fs, root_node, "height");
		calib_x_res = width_node->data.i;
		calib_y_res = height_node->data.i;
		cvReleaseFileStorage(&fs); 
		return true;
	}
	// reset error status
	cvSetErrStatus(CV_StsOk);
	return false;
}

void Camera::camInfoCallback (const sensor_msgs::CameraInfoConstPtr & cam_info)
  {
    if (!getCamInfo_)
    {
		cam_info_ = (*cam_info);

		calib_x_res = cam_info_.width;
		calib_y_res = cam_info_.height;
		x_res = calib_x_res;
		y_res = calib_y_res;

		cvmSet(&calib_K, 0, 0, cam_info_.K[0]);
		cvmSet(&calib_K, 0, 1, cam_info_.K[1]);
		cvmSet(&calib_K, 0, 2, cam_info_.K[2]);
		cvmSet(&calib_K, 1, 0, cam_info_.K[3]);
		cvmSet(&calib_K, 1, 1, cam_info_.K[4]);
		cvmSet(&calib_K, 1, 2, cam_info_.K[5]);
		cvmSet(&calib_K, 2, 0, cam_info_.K[6]);
		cvmSet(&calib_K, 2, 1, cam_info_.K[7]);
		cvmSet(&calib_K, 2, 2, cam_info_.K[8]);

		cvmSet(&calib_D, 0, 0, cam_info_.D[0]);
		cvmSet(&calib_D, 1, 0, cam_info_.D[1]);
		cvmSet(&calib_D, 2, 0, cam_info_.D[2]);
		cvmSet(&calib_D, 3, 0, cam_info_.D[3]);      	
		  
		getCamInfo_ = true;
    }
  }

bool Camera::SetCalib(const char *calibfile, int _x_res, int _y_res, FILE_FORMAT format) {
	x_res = _x_res;
	y_res = _y_res;
	if(!calibfile) return false;

	bool success = false;
	switch (format) {
		case FILE_FORMAT_XML:
			success = LoadCalibXML(calibfile);
			break;
		case FILE_FORMAT_OPENCV:
		case FILE_FORMAT_DEFAULT:
			success = LoadCalibOpenCV(calibfile);
			break;
		default:
			// TODO: throw exception?
			break;
	};

	if (success) {
		// Scale matrix in case of different resolution calibration.
		// The OpenCV documentation says:
		// - If an image from camera is up-sampled/down-sampled by some factor, all intrinsic camera parameters 
		//   (fx, fy, cx and cy) should be scaled (multiplied/divided, respectively) by the same factor.
		// - The distortion coefficients remain the same regardless of the captured image resolution.
		if ((calib_x_res != x_res) || (calib_y_res != y_res)) {
			calib_K_data[0][0] *= (double(x_res)/double(calib_x_res));
			calib_K_data[0][2] *= (double(x_res)/double(calib_x_res));
			calib_K_data[1][1] *= (double(y_res)/double(calib_y_res));
			calib_K_data[1][2] *= (double(y_res)/double(calib_y_res));
		}
	}
	return success;
}

bool Camera::SaveCalibXML(const char *calibfile) {
	TiXmlDocument document;
	document.LinkEndChild(new TiXmlDeclaration("1.0", "UTF-8", "no"));
	document.LinkEndChild(new TiXmlElement("camera"));
	TiXmlElement *xml_root = document.RootElement();
	xml_root->SetAttribute("width", calib_x_res);
	xml_root->SetAttribute("height", calib_y_res);
	xml_root->LinkEndChild(FileFormatUtils::createXMLMatrix("intrinsic_matrix", &calib_K));
	xml_root->LinkEndChild(FileFormatUtils::createXMLMatrix("distortion", &calib_D));
	return document.SaveFile(calibfile);
}

bool Camera::SaveCalibOpenCV(const char *calibfile) {
	cvSetErrMode(CV_ErrModeSilent);
	CvFileStorage* fs = cvOpenFileStorage(calibfile, 0, CV_STORAGE_WRITE); 
	cvSetErrMode(CV_ErrModeLeaf);
	if(fs){
		cvWrite(fs, "intrinsic_matrix", &calib_K, cvAttrList(0,0)); 
		cvWrite(fs, "distortion", &calib_D, cvAttrList(0,0)); 
		//cvWriteReal(fs, "fov_x", data.fov_x); 
		//cvWriteReal(fs, "fov_y", data.fov_y); 
		cvWriteInt(fs, "width", calib_x_res);
		cvWriteInt(fs, "height", calib_y_res);
		cvReleaseFileStorage(&fs); 
		return true;
	}
	// reset error status
	cvSetErrStatus(CV_StsOk);
	return false;
}

bool Camera::SaveCalib(const char *calibfile, FILE_FORMAT format) {
	if(!calibfile)
		return false;

	switch (format) {
		case FILE_FORMAT_XML:
			return SaveCalibXML(calibfile);
		case FILE_FORMAT_OPENCV:
		case FILE_FORMAT_DEFAULT:
			return SaveCalibOpenCV(calibfile);
		default:
			return false;
	};
}

void Camera::Calibrate(ProjPoints &pp)
{
	CvMat *object_points = cvCreateMat((int)pp.object_points.size(), 1, CV_32FC3);
	CvMat *image_points = cvCreateMat((int)pp.image_points.size(), 1, CV_32FC2);
	const CvMat point_counts = cvMat((int)pp.point_counts.size(), 1, CV_32SC1, &pp.point_counts[0]);
	for (size_t i=0; i<pp.object_points.size(); i++) {
		object_points->data.fl[i*3+0] = (float)pp.object_points[i].x;
		object_points->data.fl[i*3+1] = (float)pp.object_points[i].y;
		object_points->data.fl[i*3+2] = (float)pp.object_points[i].z;
		image_points->data.fl[i*2+0]  = (float)pp.image_points[i].x;
		image_points->data.fl[i*2+1]  = (float)pp.image_points[i].y;
	}
	cvCalibrateCamera2(object_points, image_points, &point_counts, 
		cvSize(pp.width, pp.height), 
		&calib_K, &calib_D, 0, 0, CV_CALIB_USE_INTRINSIC_GUESS);

	calib_x_res = pp.width;
	calib_y_res = pp.height;
	
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
}

void Camera::SetRes(int _x_res, int _y_res) {
	x_res = _x_res;
	y_res = _y_res;
	// Scale matrix in case of different resolution calibration.
	// The OpenCV documentation says:
	// - If an image from camera is up-sampled/down-sampled by some factor, all intrinsic camera parameters 
	//   (fx, fy, cx and cy) should be scaled (multiplied/divided, respectively) by the same factor.
	// - The distortion coefficients remain the same regardless of the captured image resolution.
	if ((calib_x_res != x_res) || (calib_y_res != y_res)) {
		calib_K_data[0][0] *= (double(x_res)/double(calib_x_res));
		calib_K_data[0][2] *= (double(x_res)/double(calib_x_res));
		calib_K_data[1][1] *= (double(y_res)/double(calib_y_res));
		calib_K_data[1][2] *= (double(y_res)/double(calib_y_res));
	}
}

// TODO: Better approach for this...
// Note, the proj_matrix[8] is now negated. This is due to the fact
// that with OpenCV and OpenGL projection matrices both y and z
// should be mirrored. All other components are 
void Camera::GetOpenglProjectionMatrix(double proj_matrix[16], const int width, const int height, const float far_clip /*= 1000.0f*/, const float near_clip /*= 0.1f*/) {
	proj_matrix[0]	= 2.0f * calib_K_data[0][0] / float(width);
	proj_matrix[1]	= 0;
	proj_matrix[2]	= 0;
	proj_matrix[3]	= 0;
	proj_matrix[4]  = 2.0f * calib_K_data[0][1] / float(width); // skew
	proj_matrix[5]	= 2.0f * calib_K_data[1][1] / float(height);
	proj_matrix[6]	= 0;
	proj_matrix[7]	= 0;
	//proj_matrix[8]	= (2.0f * calib_K_data[0][2] / float(width)) - 1.0f;
	proj_matrix[8]	= -(2.0f * calib_K_data[0][2] / float(width)) + 1.0f;
	proj_matrix[9]	= (2.0f * calib_K_data[1][2] / float(height)) - 1.0f;
	proj_matrix[10]	= -(far_clip + near_clip)/(far_clip - near_clip);
	proj_matrix[11]	= -1.0f;
	proj_matrix[12]	= 0;
	proj_matrix[13]	= 0;
	proj_matrix[14]	= -2.0f * far_clip * near_clip / (far_clip - near_clip);
	proj_matrix[15]	= 0;
}

void Camera::SetOpenglProjectionMatrix(double proj_matrix[16], const int width, const int height) {
	x_res = width;
	y_res = height;
	calib_x_res = width;
	calib_y_res = height;
	calib_K_data[0][0] = proj_matrix[0] * float(width) / 2.0f;
	calib_K_data[0][1] = proj_matrix[4] * float(width) / 2.0f;
	calib_K_data[1][1] = proj_matrix[5] * float(height) / 2.0f;
	//calib_K_data[0][2] = (proj_matrix[8] + 1.0f) * float(width) / 2.0f;
	calib_K_data[0][2] = (-proj_matrix[8] + 1.0f) * float(width) / 2.0f; // Is this ok?
	calib_K_data[1][2] = (proj_matrix[9] + 1.0f) * float(height) / 2.0f;
	calib_K_data[2][2] = 1;
}

void Camera::Undistort(PointDouble &point)
{
/*
	// focal length
	double ifx = 1./cvmGet(&calib_K, 0, 0);
	double ify = 1./cvmGet(&calib_K, 1, 1);

	// principal point
	double cx = cvmGet(&calib_K, 0, 2);
	double cy = cvmGet(&calib_K, 1, 2);

	// distortion coeffs
	double* k = calib_D.data.db;

	// compensate distortion iteratively
	double x = (point.x - cx)*ifx, y = (point.y - cy)*ify, x0 = x, y0 = y;
	for(int j = 0; j < 5; j++){
		double r2 = x*x + y*y;
		double icdist = 1./(1 + k[0]*r2 + k[1]*r2*r2);
		double delta_x = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
		double delta_y = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
		x = (x0 - delta_x)*icdist;
		y = (y0 - delta_y)*icdist;
	}
	// apply compensation
	point.x = x/ifx + cx;
	point.y = y/ify + cy;
*/
}

void Camera::Undistort(vector<PointDouble >& points)
{
/*
	// focal length
	double ifx = 1./cvmGet(&calib_K, 0, 0);
	double ify = 1./cvmGet(&calib_K, 1, 1);

	// principal point
	double cx = cvmGet(&calib_K, 0, 2);
	double cy = cvmGet(&calib_K, 1, 2);

	// distortion coeffs
	double* k = calib_D.data.db;

	for(unsigned int i = 0; i < points.size(); i++)
	{
		// compensate distortion iteratively
		double x = (points[i].x - cx)*ifx, y = (points[i].y - cy)*ify, x0 = x, y0 = y;
		for(int j = 0; j < 5; j++){
			double r2 = x*x + y*y;
			double icdist = 1./(1 + k[0]*r2 + k[1]*r2*r2);
			double delta_x = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
			double delta_y = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
			x = (x0 - delta_x)*icdist;
			y = (y0 - delta_y)*icdist;
		}
		// apply compensation
		points[i].x = x/ifx + cx;
		points[i].y = y/ify + cy;
	}
*/
}

void Camera::Undistort(CvPoint2D32f& point)
{
/*
	// focal length
	double ifx = 1./cvmGet(&calib_K, 0, 0);
	double ify = 1./cvmGet(&calib_K, 1, 1);

	// principal point
	double cx = cvmGet(&calib_K, 0, 2);
	double cy = cvmGet(&calib_K, 1, 2);

	// distortion coeffs
	double* k = calib_D.data.db;


	// compensate distortion iteratively
	double x = (point.x - cx)*ifx, y = (point.y - cy)*ify, x0 = x, y0 = y;
	for(int j = 0; j < 5; j++){
		double r2 = x*x + y*y;
		double icdist = 1./(1 + k[0]*r2 + k[1]*r2*r2);
		double delta_x = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
		double delta_y = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
		x = (x0 - delta_x)*icdist;
		y = (y0 - delta_y)*icdist;
	}
	// apply compensation
	point.x = float(x/ifx + cx);
	point.y = float(y/ify + cy);
*/
}

/*
	template<class PointType>
	void Undistort(PointType& point) {
		// focal length
		double ifx = 1./cvmGet(&calib_K, 0, 0);
		double ify = 1./cvmGet(&calib_K, 1, 1);

		// principal point
		double cx = cvmGet(&calib_K, 0, 2);
		double cy = cvmGet(&calib_K, 1, 2);

		// distortion coeffs
		double* k = calib_D.data.db;

		// compensate distortion iteratively
		double x = (point.x - cx)*ifx, y = (point.y - cy)*ify, x0 = x, y0 = y;
		for(int j = 0; j < 5; j++){
			double r2 = x*x + y*y;
			double icdist = 1./(1 + k[0]*r2 + k[1]*r2*r2);
			double delta_x = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
			double delta_y = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
			x = (x0 - delta_x)*icdist;
			y = (y0 - delta_y)*icdist;
		}
		// apply compensation
		point.x = x/ifx + cx;
		point.y = y/ify + cy;
	}
*/

void Camera::Distort(vector<PointDouble>& points) 
{
/*
	double u0 = cvmGet(&calib_K, 0, 2), v0 = cvmGet(&calib_K, 1, 2); // cx, cy
	double fx = cvmGet(&calib_K, 0, 0), fy = cvmGet(&calib_K, 1, 1);
	double _fx = 1./fx, _fy = 1./fy;
	double* k = calib_D.data.db;

	double k1 = k[0], k2 = k[1];
	double p1 = k[2], p2 = k[3];

	for(unsigned int i = 0; i < points.size(); i++)
	{
		// Distort
		double y = (points[i].y - v0)*_fy;
		double y2 = y*y;
		double _2p1y = 2*p1*y;
		double _3p1y2 = 3*p1*y2;
		double p2y2 = p2*y2;

		double x = (points[i].x - u0)*_fx;
		double x2 = x*x;
		double r2 = x2 + y2;
		double d = 1 + (k1 + k2*r2)*r2;

		points[i].x = fx*(x*(d + _2p1y) + p2y2 + (3*p2)*x2) + u0;
		points[i].y = fy*(y*(d + (2*p2)*x) + _3p1y2 + p1*x2) + v0;
	}
*/
}

void Camera::Distort(PointDouble & point) 
{
/*
	double u0 = cvmGet(&calib_K, 0, 2), v0 = cvmGet(&calib_K, 1, 2); // cx, cy
	double fx = cvmGet(&calib_K, 0, 0), fy = cvmGet(&calib_K, 1, 1);
	double _fx = 1./fx, _fy = 1./fy;
	double* k = calib_D.data.db;

	double k1 = k[0], k2 = k[1];
	double p1 = k[2], p2 = k[3];

	// Distort
	double y = (point.y - v0)*_fy;
	double y2 = y*y;
	double _2p1y = 2*p1*y;
	double _3p1y2 = 3*p1*y2;
	double p2y2 = p2*y2;

	double x = (point.x - u0)*_fx;
	double x2 = x*x;
	double r2 = x2 + y2;
	double d = 1 + (k1 + k2*r2)*r2;

	point.x = fx*(x*(d + _2p1y) + p2y2 + (3*p2)*x2) + u0;
	point.y = fy*(y*(d + (2*p2)*x) + _3p1y2 + p1*x2) + v0;
*/
}

void Camera::Distort(CvPoint2D32f & point) 
{
/*
	double u0 = cvmGet(&calib_K, 0, 2), v0 = cvmGet(&calib_K, 1, 2); // cx, cy
	double fx = cvmGet(&calib_K, 0, 0), fy = cvmGet(&calib_K, 1, 1);
	double _fx = 1./fx, _fy = 1./fy;
	double* k = calib_D.data.db;

	double k1 = k[0], k2 = k[1];
	double p1 = k[2], p2 = k[3];

	// Distort
	double y = (point.y - v0)*_fy;
	double y2 = y*y;
	double _2p1y = 2*p1*y;
	double _3p1y2 = 3*p1*y2;
	double p2y2 = p2*y2;

	double x = (point.x - u0)*_fx;
	double x2 = x*x;
	double r2 = x2 + y2;
	double d = 1 + (k1 + k2*r2)*r2;

	point.x = float(fx*(x*(d + _2p1y) + p2y2 + (3*p2)*x2) + u0);
	point.y = float(fy*(y*(d + (2*p2)*x) + _3p1y2 + p1*x2) + v0);
*/
}

void Camera::CalcExteriorOrientation(vector<CvPoint3D64f>& pw, vector<CvPoint2D64f>& pi,
					Pose *pose)
{
	double ext_rodriques[3];
	double ext_translate[3];
	CvMat ext_rodriques_mat = cvMat(3, 1, CV_64F, ext_rodriques);
	CvMat ext_translate_mat = cvMat(3, 1, CV_64F, ext_translate);
	CvMat *object_points = cvCreateMat((int)pw.size(), 1, CV_32FC3);
	CvMat *image_points = cvCreateMat((int)pi.size(), 1, CV_32FC2);
	for (size_t i=0; i<pw.size(); i++) {
		object_points->data.fl[i*3+0] = (float)pw[i].x;
		object_points->data.fl[i*3+1] = (float)pw[i].y;
		object_points->data.fl[i*3+2] = (float)pw[i].z;
		image_points->data.fl[i*2+0]  = (float)pi[i].x;
		image_points->data.fl[i*2+1]  = (float)pi[i].y;
	}
	//cvFindExtrinsicCameraParams2(object_points, image_points, &calib_K, &calib_D, &ext_rodriques_mat, &ext_translate_mat);
	cvFindExtrinsicCameraParams2(object_points, image_points, &calib_K, NULL, &ext_rodriques_mat, &ext_translate_mat);
    pose->SetRodriques(&ext_rodriques_mat);
	pose->SetTranslation(&ext_translate_mat);
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
}

void Camera::CalcExteriorOrientation(vector<CvPoint3D64f>& pw, vector<PointDouble >& pi,
					CvMat *rodriques, CvMat *tra)
{
	//assert(pw.size() == pi.size());

	int size = (int)pi.size();

	CvPoint3D64f *world_pts = new CvPoint3D64f[size];
	CvPoint2D64f *image_pts = new CvPoint2D64f[size];

	for(int i = 0; i < size; i++){
		world_pts[i].x = pw[i].x;
		world_pts[i].y = pw[i].y;
		world_pts[i].z = pw[i].z;
		// flip image points! Why???
		//image_pts[i].x = x_res - pi[i].x;
		//image_pts[i].y = y_res - pi[i].y;
		image_pts[i].x = pi[i].x;
		image_pts[i].y = pi[i].y;
	}

	double rot[3]; // rotation vector
	CvMat world_mat, image_mat, rot_vec;
	cvInitMatHeader(&world_mat, size, 1, CV_64FC3, world_pts);
	cvInitMatHeader(&image_mat, size, 1, CV_64FC2, image_pts);
	cvInitMatHeader(&rot_vec, 3, 1, CV_64FC1, rot);

	cvZero(tra);
	//cvmodFindExtrinsicCameraParams2(&world_mat, &image_mat, &calib_K, &calib_D, rodriques, tra, error);
	cvFindExtrinsicCameraParams2(&world_mat, &image_mat, &calib_K, &calib_D, rodriques, tra);
	
	delete[] world_pts;
	delete[] image_pts;
}

void Camera::CalcExteriorOrientation(vector<PointDouble >& pw, vector<PointDouble >& pi,
					CvMat *rodriques, CvMat *tra)
{
	//assert(pw.size() == pi.size());
	int size = (int)pi.size();

	vector<CvPoint3D64f> pw3;
	pw3.resize(size);
	for (int i=0; i<size; i++) {
		pw3[i].x = pw[i].x;
		pw3[i].y = pw[i].y;
		pw3[i].z = 0;
	}

	CalcExteriorOrientation(pw3, pi, rodriques, tra);
}

void Camera::CalcExteriorOrientation(vector<PointDouble>& pw, vector<PointDouble >& pi, Pose *pose)
{
	double ext_rodriques[3];
	double ext_translate[3];
	CvMat ext_rodriques_mat = cvMat(3, 1, CV_64F, ext_rodriques);
	CvMat ext_translate_mat = cvMat(3, 1, CV_64F, ext_translate);
	CalcExteriorOrientation(pw, pi, &ext_rodriques_mat, &ext_translate_mat);
	pose->SetRodriques(&ext_rodriques_mat);
	pose->SetTranslation(&ext_translate_mat);
}

bool Camera::CalcExteriorOrientation(const CvMat* object_points, CvMat* image_points, CvMat *rodriques, CvMat *tra) {
	cvFindExtrinsicCameraParams2(object_points, image_points, &calib_K, &calib_D, rodriques, tra);
	return true;
}

bool Camera::CalcExteriorOrientation(const CvMat* object_points, CvMat* image_points, Pose *pose) {
	double ext_rodriques[3];
	double ext_translate[3];
	CvMat ext_rodriques_mat = cvMat(3, 1, CV_64F, ext_rodriques);
	CvMat ext_translate_mat = cvMat(3, 1, CV_64F, ext_translate);
	bool ret = CalcExteriorOrientation(object_points, image_points, &ext_rodriques_mat, &ext_translate_mat);
	pose->SetRodriques(&ext_rodriques_mat);
	pose->SetTranslation(&ext_translate_mat);
	return ret;
}

void Camera::ProjectPoints(vector<CvPoint3D64f>& pw, Pose *pose, vector<CvPoint2D64f>& pi) const {
	double ext_rodriques[3];
	double ext_translate[3];
	CvMat ext_rodriques_mat = cvMat(3, 1, CV_64F, ext_rodriques);
	CvMat ext_translate_mat = cvMat(3, 1, CV_64F, ext_translate);
	pose->GetRodriques(&ext_rodriques_mat);
	pose->GetTranslation(&ext_translate_mat);
	CvMat *object_points = cvCreateMat((int)pw.size(), 1, CV_32FC3);
	CvMat *image_points = cvCreateMat((int)pi.size(), 1, CV_32FC2);
	for (size_t i=0; i<pw.size(); i++) {
		object_points->data.fl[i*3+0] = (float)pw[i].x;
		object_points->data.fl[i*3+1] = (float)pw[i].y;
		object_points->data.fl[i*3+2] = (float)pw[i].z;
	}
	cvProjectPoints2(object_points, &ext_rodriques_mat, &ext_translate_mat, &calib_K, &calib_D, image_points);  
	for (size_t i=0; i<pw.size(); i++) {
		pi[i].x = image_points->data.fl[i*2+0];
		pi[i].y = image_points->data.fl[i*2+1];
	}
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
}

void Camera::ProjectPoints(const CvMat* object_points, const CvMat* rotation_vector,
				   const CvMat* translation_vector, CvMat* image_points) const
{
	// Project points
	cvProjectPoints2(object_points, rotation_vector, translation_vector, &calib_K, &calib_D, image_points);  
}

void Camera::ProjectPoints(const CvMat* object_points, const Pose* pose, CvMat* image_points) const
{
	double ext_rodriques[3];
	double ext_translate[3];
	CvMat ext_rodriques_mat = cvMat(3, 1, CV_64F, ext_rodriques);
	CvMat ext_translate_mat = cvMat(3, 1, CV_64F, ext_translate);
	pose->GetRodriques(&ext_rodriques_mat);
	pose->GetTranslation(&ext_translate_mat);
	cvProjectPoints2(object_points, &ext_rodriques_mat, &ext_translate_mat, &calib_K, &calib_D, image_points);  
}

void Camera::ProjectPoints(const CvMat* object_points, double gl[16], CvMat* image_points) const
{
	double glm[4][4] = {
		gl[0], gl[4], gl[8],  gl[12],
		gl[1], gl[5], gl[9],  gl[13],
		gl[2], gl[6], gl[10], gl[14],
		gl[3], gl[7], gl[11], gl[15],
	};
	CvMat glm_mat = cvMat(4, 4, CV_64F, glm);
	
	// For some reason we need to mirror both y and z ???
	double cv_mul_data[4][4];
	CvMat cv_mul = cvMat(4, 4, CV_64F, cv_mul_data);
	cvSetIdentity(&cv_mul);
	cvmSet(&cv_mul, 1, 1, -1);
	cvmSet(&cv_mul, 2, 2, -1);
	cvMatMul(&cv_mul, &glm_mat, &glm_mat);
	
	// Rotation
	Rotation r;
	r.SetMatrix(&glm_mat);
	double rod[3]; 
	CvMat rod_mat=cvMat(3, 1, CV_64F, rod);
	r.GetRodriques(&rod_mat);
	// Translation
	double tra[3] = { glm[0][3], glm[1][3], glm[2][3] };
	CvMat tra_mat = cvMat(3, 1, CV_64F, tra);
	// Project points
	ProjectPoints(object_points, &rod_mat, &tra_mat, image_points);
}

void Camera::ProjectPoint(const CvPoint3D64f pw, const Pose *pose, CvPoint2D64f &pi) const {
	float object_points_data[3] = {(float)pw.x, (float)pw.y, (float)pw.z};
	float image_points_data[2] = {0};
	CvMat object_points = cvMat(1, 1, CV_32FC3, object_points_data);
	CvMat image_points = cvMat(1, 1, CV_32FC2, image_points_data);
	ProjectPoints(&object_points, pose, &image_points);
	pi.x = image_points.data.fl[0];
	pi.y = image_points.data.fl[1];
}

void Camera::ProjectPoint(const CvPoint3D32f pw, const Pose *pose, CvPoint2D32f &pi) const {
	float object_points_data[3] = {(float)pw.x, (float)pw.y, (float)pw.z};
	float image_points_data[2] = {0};
	CvMat object_points = cvMat(1, 1, CV_32FC3, object_points_data);
	CvMat image_points = cvMat(1, 1, CV_32FC2, image_points_data);
	ProjectPoints(&object_points, pose, &image_points);
	pi.x = image_points.data.fl[0];
	pi.y = image_points.data.fl[1];
}

Homography::Homography() {
	cvInitMatHeader(&H, 3, 3, CV_64F, H_data);
}

void Homography::Find(const vector<PointDouble  >& pw, const vector<PointDouble  >& pi)
{
	assert(pw.size() == pi.size());
	int size = (int)pi.size();

	CvPoint2D64f *srcp = new CvPoint2D64f[size];
	CvPoint2D64f *dstp = new CvPoint2D64f[size];

	for(int i = 0; i < size; ++i){
		srcp[i].x = pw[i].x;
		srcp[i].y = pw[i].y;

		dstp[i].x = pi[i].x;
		dstp[i].y = pi[i].y;
	}
	
	CvMat src_pts, dst_pts;
	cvInitMatHeader(&dst_pts, 1, size, CV_64FC2, dstp);
	cvInitMatHeader(&src_pts, 1, size, CV_64FC2, srcp);

#ifdef OPENCV11
	cvFindHomography(&src_pts, &dst_pts, &H, 0, 0, 0);
#else
	cvFindHomography(&src_pts, &dst_pts, &H);
#endif

	delete[] srcp;
	delete[] dstp;
}

void Homography::ProjectPoints(const vector<PointDouble>& from, vector<PointDouble>& to) 
{
	int size = (int)from.size();

	CvPoint3D64f *srcp = new CvPoint3D64f[size];

	for(int i = 0; i < size; ++i){
		srcp[i].x = from[i].x;
		srcp[i].y = from[i].y;
		srcp[i].z = 1;
	}

	CvPoint3D64f *dstp = new CvPoint3D64f[size];

	CvMat src_pts, dst_pts;
	cvInitMatHeader(&src_pts, 1, size, CV_64FC3, srcp);
	cvInitMatHeader(&dst_pts, 1, size, CV_64FC3, dstp);
	
	cvTransform(&src_pts, &dst_pts, &H);

	to.clear();
	for(int i = 0; i < size; ++i)
	{	
		PointDouble pt;
		pt.x = dstp[i].x / dstp[i].z;
		pt.y = dstp[i].y / dstp[i].z;
		
		to.push_back(pt);
	}
	
	delete[] srcp;
	delete[] dstp;
}

} // namespace alvar

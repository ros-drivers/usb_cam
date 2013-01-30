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

#include <iostream>
#include <algorithm> // for std::max
#include "cxcore.h"
#include "cv.h"
#include "highgui.h"
#include "Kalman.h"
#include "Util.h"
#include "Alvar.h"

namespace alvar {

KalmanSensorCore::KalmanSensorCore(const KalmanSensorCore &k) {
	m = k.m;
	n = k.n;
	z = cvCloneMat(k.z);
	H = cvCloneMat(k.H);
	H_trans = cvCloneMat(k.H_trans);
	K = cvCloneMat(k.K);
	z_pred = cvCloneMat(k.z_pred);
	z_residual = cvCloneMat(k.z_residual);
	x_gain = cvCloneMat(k.x_gain);
}

KalmanSensorCore::KalmanSensorCore(int _n, int _m) {
	n = _n;
	m = _m;
	z = cvCreateMat(m,1,CV_64FC1); cvSetZero(z);
	H = cvCreateMat(m,n,CV_64FC1); cvSetZero(H);
	H_trans = cvCreateMat(n,m,CV_64FC1); cvSetZero(H_trans);
	K = cvCreateMat(n,m,CV_64FC1); cvSetZero(K);
	z_pred = cvCreateMat(m,1,CV_64FC1); cvSetZero(z_pred);
	z_residual = cvCreateMat(m,1,CV_64FC1); cvSetZero(z_residual);
	x_gain = cvCreateMat(n,1,CV_64FC1); cvSetZero(x_gain);
}

KalmanSensorCore::~KalmanSensorCore() {
	cvReleaseMat(&z);
	cvReleaseMat(&H);
	cvReleaseMat(&H_trans);
	cvReleaseMat(&K);
	cvReleaseMat(&z_pred);
	cvReleaseMat(&z_residual);
	cvReleaseMat(&x_gain);
}

void KalmanSensorCore::update_x(CvMat *x_pred, CvMat *x) {
	// x = x_pred + K * (z - H*x_pred)
	cvMatMul(H, x_pred, z_pred);
	cvScaleAdd(z_pred, cvScalar(-1), z, z_residual);
	cvMatMul(K, z_residual, x_gain);
	cvScaleAdd(x_pred, cvScalar(1), x_gain, x);
}

void KalmanCore::predict_x(unsigned long tick) {
	// x_pred = F * x;
	cvMatMul(F, x, x_pred);
}

KalmanCore::KalmanCore(const KalmanCore &s) {
	n = s.n;
	x = cvCloneMat(s.x);
	F = cvCloneMat(s.F);
	x_pred = cvCloneMat(s.x_pred);
	F_trans = cvCloneMat(s.F_trans);
}

KalmanCore::KalmanCore(int _n) {
	n = _n;
	x = cvCreateMat(n,1,CV_64FC1); cvSetZero(x);
	F = cvCreateMat(n,n,CV_64FC1); cvSetIdentity(F);
	F_trans = cvCreateMat(n,n,CV_64FC1); cvSetIdentity(F_trans);
	x_pred = cvCreateMat(n,1,CV_64FC1); cvSetZero(x_pred);
}

KalmanCore::~KalmanCore() {
	cvReleaseMat(&x);
	cvReleaseMat(&F);
	cvReleaseMat(&F_trans);
	cvReleaseMat(&x_pred);
}

CvMat *KalmanCore::predict() {
	predict_x(0);
	return x_pred;
}

CvMat *KalmanCore::predict_update(KalmanSensorCore *sensor) {
	predict();
	sensor->update_x(x_pred, x);
	return x;
}

KalmanSensor::KalmanSensor(const KalmanSensor &k) : KalmanSensorCore(k) {
	R = cvCloneMat(k.R);
	R_tmp = cvCloneMat(k.R_tmp);
	P_tmp = cvCloneMat(k.P_tmp);
}

KalmanSensor::KalmanSensor(int n, int _m) : KalmanSensorCore(n, _m) {
	R = cvCreateMat(m,m,CV_64FC1); cvSetZero(R);
	R_tmp = cvCreateMat(m,m,CV_64FC1); cvSetZero(R);
	P_tmp = cvCreateMat(n,n,CV_64FC1); cvSetZero(P_tmp);
}

KalmanSensor::~KalmanSensor() {
	cvReleaseMat(&R);
	cvReleaseMat(&R_tmp);
	cvReleaseMat(&P_tmp);
}

void KalmanSensor::update_K(CvMat *P_pred) {
	// K = P * trans(H) * inv(H*P*trans(H) + R)
	cvTranspose(H, H_trans);
	cvMatMul(P_pred, H_trans, K);
	cvMatMul(H, K, R_tmp);
	cvScaleAdd(R_tmp, cvScalar(1), R, R_tmp);
	cvInvert(R_tmp, R_tmp);
	cvMatMul(H_trans, R_tmp, K);
	cvMatMul(P_pred, K, K);
}

void KalmanSensor::update_P(CvMat *P_pred, CvMat *P) {
	//P = (I - K*H) * P_pred
	cvMatMul(K, H, P_tmp);
	cvSetIdentity(P);
	cvScaleAdd(P_tmp, cvScalar(-1), P, P);
	cvMatMul(P, P_pred, P);
}

void Kalman::predict_P() {
	// P_pred = F*P*trans(F) + Q
	cvTranspose(F, F_trans);
	cvMatMul(P, F_trans, P_pred);
	cvMatMul(F, P_pred, P_pred);
	cvScaleAdd(P_pred, cvScalar(1), Q, P_pred);
}

Kalman::Kalman(int _n) : KalmanCore(_n) {
	prev_tick = 0;
	Q = cvCreateMat(n,n,CV_64FC1); cvSetZero(Q);
	P = cvCreateMat(n,n,CV_64FC1); cvSetZero(P);
	P_pred = cvCreateMat(n,n,CV_64FC1); cvSetZero(P_pred);
}

Kalman::~Kalman() {
	cvReleaseMat(&Q);
	cvReleaseMat(&P);
	cvReleaseMat(&P_pred);
}

void Kalman::update_F(unsigned long tick) {
	//cvSetIdentity(F);
}

CvMat *Kalman::predict(unsigned long tick) {
	update_F(tick);
	predict_x(tick);
	predict_P();
	return x_pred;
}

CvMat *Kalman::predict_update(KalmanSensor *sensor, unsigned long tick) {
	predict(tick);
	sensor->update_H(x_pred);
	sensor->update_K(P_pred);
	sensor->update_x(x_pred, x);
	sensor->update_P(P_pred, P);
	prev_tick = tick;
	return x;
}

double Kalman::seconds_since_update(unsigned long tick) {
	unsigned long tick_diff = (prev_tick ? tick-prev_tick : 0);
	return ((double)tick_diff/1000.0);
}

void KalmanSensorEkf::update_H(CvMat *x_pred) {
	// By default we update the H by calculating Jacobian numerically
	const double step = 0.000001;
	cvZero(H);
	for (int i=0; i<n; i++) {
		CvMat H_column;
		cvGetCol(H, &H_column, i);

		cvZero(delta); 
		cvmSet(delta, i, 0, step);
		cvAdd(x_pred, delta, x_plus);
		cvmSet(delta, i, 0, -step);
		cvAdd(x_pred, delta, x_minus);

		h(x_plus, z_tmp1);  
		h(x_minus, z_tmp2);	
		cvSub(z_tmp1, z_tmp2, &H_column);
		cvScale(&H_column, &H_column, 1.0/(2*step));
	}
}

void KalmanSensorEkf::update_x(CvMat *x_pred, CvMat *x) {
	// x = x_pred + K * (z - h(x_pred))
	h(x_pred, z_pred);
	cvScaleAdd(z_pred, cvScalar(-1), z, z_residual);
	cvMatMul(K, z_residual, x_gain);
	cvScaleAdd(x_pred, cvScalar(1), x_gain, x);
}

KalmanSensorEkf::KalmanSensorEkf(const KalmanSensorEkf &k) : KalmanSensor(k) {
	delta   = cvCloneMat(k.delta);
	x_plus  = cvCloneMat(k.x_plus);
	x_minus = cvCloneMat(k.x_minus);
	z_tmp1  = cvCloneMat(k.z_tmp1);
	z_tmp2  = cvCloneMat(k.z_tmp2);
}

KalmanSensorEkf::KalmanSensorEkf(int _n, int _m) : KalmanSensor(_n, _m) {
	delta = cvCreateMat(n,1,CV_64FC1); cvSetZero(delta);
	x_plus = cvCreateMat(n,1,CV_64FC1); cvSetZero(x_plus);
	x_minus = cvCreateMat(n,1,CV_64FC1); cvSetZero(x_minus);
	z_tmp1 = cvCreateMat(m,1,CV_64FC1); cvSetZero(z_tmp1);
	z_tmp2 = cvCreateMat(m,1,CV_64FC1); cvSetZero(z_tmp2);
}

KalmanSensorEkf::~KalmanSensorEkf() {
	cvReleaseMat(&delta);
	cvReleaseMat(&x_plus);
	cvReleaseMat(&x_minus);
	cvReleaseMat(&z_tmp1);
	cvReleaseMat(&z_tmp2);
}

void KalmanEkf::update_F(unsigned long tick) {
	// By default we update the F by calculating Jacobian numerically
	// TODO
	double dt = (tick-prev_tick)/1000.0;
	const double step = 0.000001;
	cvZero(F);
	for (int i=0; i<n; i++) {
		CvMat F_column;
		cvGetCol(F, &F_column, i);

		cvZero(delta); 
		cvmSet(delta, i, 0, step);
		cvAdd(x, delta, x_plus);
		cvmSet(delta, i, 0, -step);
		cvAdd(x, delta, x_minus);

		f(x_plus, x_tmp1, dt);  
		f(x_minus, x_tmp2, dt);	
		cvSub(x_tmp1, x_tmp2, &F_column);
		cvScale(&F_column, &F_column, 1.0/(2*step));
	}
}

void KalmanEkf::predict_x(unsigned long tick) {
	double dt = (tick-prev_tick)/1000.0;
	f(x, x_pred, dt);
}

KalmanEkf::KalmanEkf(int _n) : Kalman(_n) {
	delta = cvCreateMat(n,1,CV_64FC1); cvSetZero(delta);
	x_plus = cvCreateMat(n,1,CV_64FC1); cvSetZero(x_plus);
	x_minus = cvCreateMat(n,1,CV_64FC1); cvSetZero(x_minus);
	x_tmp1 = cvCreateMat(n,1,CV_64FC1); cvSetZero(x_tmp1);
	x_tmp2 = cvCreateMat(n,1,CV_64FC1); cvSetZero(x_tmp2);
}

KalmanEkf::~KalmanEkf() {
	cvReleaseMat(&delta);
	cvReleaseMat(&x_plus);
	cvReleaseMat(&x_minus);
	cvReleaseMat(&x_tmp1);
	cvReleaseMat(&x_tmp2);
}

void KalmanVisualize::img_matrix(CvMat *mat, int top, int left) {
	cvSetImageROI(img, cvRect(top, left, mat->cols, mat->rows));
	for (int j=0; j<mat->rows; j++) {
		for (int i=0; i<mat->cols; i++) {
			double d = cvGet2D(mat, j, i).val[0];
			if (d < 0) d=-d;
			double c1=0, c2=0, c3=0;
			if (d < 0.1) {
				c1 = 0 + ((d - 0.0)/(0.1 - 0.0)*(127 - 0));
			} else if(d < 1.0) {
				c1 = 127 + ((d - 0.1)/(1.0 - 0.1)*(255 - 127));
			} else if (d < 10.0) {
				c1 = 255;
				c2 = 0 + ((d - 1.0)/(10.0 - 1.0)*(255 - 0));
			} else if (d < 100.0) {
				c1 = 255;
				c2 = 255;
				c3 = 0 + ((d - 10.0)/(100.0 - 10.0)*(255 - 0));
			} else {
				c1 = 255; c2 = 255;	c3 = 255;
			}
			if (d < 0) {
				cvSet2D(img, j, i, cvScalar(c3, c2, c1)); // BRG
			} else {
				cvSet2D(img, j, i, cvScalar(c2, c1, c3)); // BGR
			}
		}
	}
	cvResetImageROI(img);
}

void KalmanVisualize::Init() {
	n = kalman->get_n();
	m = sensor->get_m();
	int img_width = std::max(3+n+3+n+5+m+5, 1+n+1+n+1+n+1+m+1+n+1);
	int img_height = 1+n+1+std::max(n, m+1+m)+1;
	img = cvCreateImage(cvSize(img_width, img_height), IPL_DEPTH_8U, 3);
	cvSet(img, cvScalar(64,64,64));
	img_legend = cvLoadImage("Legend.png");
	if (img_legend) {
		for (img_scale=1; img_scale<50; img_scale++) {
			if (img_scale*img_width > img_legend->width) {
				break;
			}
		}
		img_show = cvCreateImage(cvSize(img_width*img_scale, img_legend->height + img_height*img_scale), IPL_DEPTH_8U, 3);
		cvSet(img_show, cvScalar(64,64,64));
		cvSetImageROI(img_show, cvRect(0, 0, img_legend->width, img_legend->height));
		cvCopy(img_legend, img_show);
		cvResetImageROI(img_show);
		cvNamedWindow("KalmanVisualize");
	} else {
		img_scale = 1;
		img_show = cvCreateImage(cvSize(img_width*img_scale, img_height*img_scale), IPL_DEPTH_8U, 3);
		cvSet(img_show, cvScalar(64,64,64));
		cvNamedWindow("KalmanVisualize",0);
	}
}

void KalmanVisualize::out_matrix(CvMat *m, char *name) {
	if (m->cols == 1) {
		std::cout<<name<<" = [";
		for (int j=0; j<m->rows; j++) {
			std::cout<<" "<<cvGet2D(m, j, 0).val[0];
		}
		std::cout<<"]^T"<<std::endl;
	} else if (m->rows == 1) {
		std::cout<<name<<" = [";
		for (int i=0; i<m->cols; i++) {
			std::cout<<" "<<cvGet2D(m, 0, i).val[0];
		}
		std::cout<<"]^T"<<std::endl;
	} else {
		std::cout<<name<<" = ["<<std::endl;
		for (int j=0; j<m->rows; j++) {
			for (int i=0; i<m->cols; i++) {
				std::cout<<" "<<cvGet2D(m, j, i).val[0];
			}
			std::cout<<std::endl;
		}
		std::cout<<"]"<<std::endl;
	}
}

KalmanVisualize::KalmanVisualize(Kalman *_kalman, KalmanSensor *_sensor) {
	kalman = _kalman;
	sensor = _sensor;
	kalman_ext = _kalman;
	sensor_ext = _sensor;
	Init();
}

KalmanVisualize::KalmanVisualize(KalmanCore *_kalman, KalmanSensorCore *_sensor) {
	kalman = _kalman;
	sensor = _sensor;
	kalman_ext = NULL;
	sensor_ext = NULL;
	Init();
}

KalmanVisualize::~KalmanVisualize() {
	cvReleaseImage(&img);
}

void KalmanVisualize::update_pre() {
	img_matrix(kalman->x, 1, 1); // 1
	if (kalman_ext && sensor_ext) {
		int y = std::max(2+n, 3+m+m);
		img_matrix(kalman_ext->P, 1, y); // n
	}
}

void KalmanVisualize::update_post() {
	img_matrix(kalman->F, 3, 1); // n
	img_matrix(kalman->x_pred, 4+n, 1); // 1
	img_matrix(sensor->H, 6+n, 1); // n
	img_matrix(sensor->z_pred, 7+n+n, 1); // 1
	img_matrix(sensor->z, 7+n+n, 2 + m);
	img_matrix(sensor->z_residual, 9+n+n, 1); // 1
	img_matrix(sensor->K, 11+n+n, 1); // m
	img_matrix(sensor->x_gain, 12+n+n+m, 1); // 1
	img_matrix(kalman->x, 14+n+n+m, 1); // 1
	if (kalman_ext && sensor_ext) {
		int y = std::max(2+n, 3+m+m);
		img_matrix(kalman_ext->Q, 2+n, y); // n
		img_matrix(kalman_ext->P_pred, 3+n+n, y); // n
		img_matrix(sensor_ext->R, 4+n+n+n, y); // m
		img_matrix(kalman_ext->P, img->width - 1 - n, y); // n
	}
	if (img_legend) {
		cvSetImageROI(img_show, cvRect(0, img_legend->height, img->width * img_scale, img->height * img_scale));
		cvResize(img, img_show, CV_INTER_NN);
		cvResetImageROI(img_show);
	} else {
		cvResize(img, img_show, CV_INTER_NN);
	}
}

void KalmanVisualize::show() {
	//out_matrix(sensor->K, "K");
	cvShowImage("KalmanVisualize", img_show);
}

} // namespace alvar

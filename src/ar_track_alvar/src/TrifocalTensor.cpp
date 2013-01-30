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

#include "TrifocalTensor.h"

namespace alvar {

TrifocalTensor::TrifocalTensor() {
}

TrifocalTensor::TrifocalTensor(const Pose &p0, const Pose &p1) {
  computeTensor(p0, p1);
}

TrifocalTensor::TrifocalTensor(const Pose &p0, const Pose &p1, const Pose &p2) {
  computeTensor(p0, p1, p2);
}

TrifocalTensor::~TrifocalTensor() {
}

void TrifocalTensor::computeTensor(const Pose &p0, const Pose &p1) {
  double data_p12[4][4], data_p13[4][4];
  CvMat p12 = cvMat( 4, 4, CV_64F, data_p12 );
  CvMat p13 = cvMat( 4, 4, CV_64F, data_p13 );
  p0.GetMatrix(&p12);
  p1.GetMatrix(&p13);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) {

	      T[i][j][k] = 
	        (data_p12[j][i] * data_p13[k][3])
	        - 
	        (data_p12[j][3] * data_p13[k][i]);
  }
}

double *getRow(double* m, int row) {
  return &m[4*row];
}

double det(double *r0, double *r1, double *r2, double *r3) {
  double m[16];
  memcpy(&m[0], r0, 4*sizeof(double));
  memcpy(&m[4], r1, 4*sizeof(double));
  memcpy(&m[8], r2, 4*sizeof(double));
  memcpy(&m[12], r3, 4*sizeof(double));
  CvMat M = cvMat(4, 4, CV_64F, m);
  return cvDet(&M);
}

void TrifocalTensor::computeTensor(const Pose &p0, const Pose &p1, const Pose &p2) {
  double data_p0[16], data_p1[16], data_p2[16];
  CvMat P0 = cvMat( 4, 4, CV_64F, data_p0 );
  CvMat P1 = cvMat( 4, 4, CV_64F, data_p1 );
  CvMat P2 = cvMat( 4, 4, CV_64F, data_p2 );
  p0.GetMatrix(&P0);
  p1.GetMatrix(&P1);
  p2.GetMatrix(&P2);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) {
        double sign = i==1 ? -1 : 1;
        T[i][j][k] = sign * det(getRow(data_p0, (i+1)%3),
                                getRow(data_p0, (i+2)%3),
                                getRow(data_p1, j),
                                getRow(data_p2, k));
      }  
}

double TrifocalTensor::projectAxis(const CvPoint2D64f &p0, const CvPoint2D64f &p1, int l) {
  double v00 =
    p1.x * (  p0.x * T[0][0][l]
	      + p0.y * T[1][0][l]
	      + T[2][0][l])
    - 
    p1.x * (  p0.x * T[0][0][l]
	      + p0.y * T[1][0][l]
	      + T[2][0][l]);
  double v01 =
    p1.x * (  p0.x * T[0][1][l]
	      + p0.y * T[1][1][l]
	      + T[2][1][l])
    - 
    p1.y * (  p0.x * T[0][0][l]
	      + p0.y * T[1][0][l]
	      + T[2][0][l]);
  double v02 =
    p1.x * (  p0.x * T[0][2][l]
	      + p0.y * T[1][2][l]
	      + T[2][2][l])
    - ( p0.x * T[0][0][l]
	+ p0.y * T[1][0][l]
	+ T[2][0][l]);
  
  double v10 =
    p1.y * (  p0.x * T[0][0][l]
	      + p0.y * T[1][0][l]
	      + T[2][0][l])
    - 
    p1.x * (  p0.x * T[0][1][l]
	      + p0.y * T[1][1][l]
	      + T[2][1][l]);
  double v11 =
    p1.y * (  p0.x * T[0][1][l]
	      + p0.y * T[1][1][l]
	      + T[2][1][l])
    - 
    p1.y * (  p0.x * T[0][1][l]
	      + p0.y * T[1][1][l]
	      + T[2][1][l]);
  double v12 =
    p1.y * (  p0.x * T[0][2][l]
	      + p0.y * T[1][2][l]
	      + T[2][2][l])
    - ( p0.x * T[0][1][l]
	+ p0.y * T[1][1][l]
	+ T[2][1][l]);
  
  double v20 =
    (  p0.x * T[0][0][l]
       + p0.y * T[1][0][l]
       + T[2][0][l])
    - 
    p1.x * (  p0.x * T[0][2][l]
	      + p0.y * T[1][2][l]
	      + T[2][2][l]);
  double v21 =
    (  p0.x * T[0][1][l]
       + p0.y * T[1][1][l]
       + T[2][1][l])
    - 
    p1.y * (  p0.x * T[0][2][l]
	      + p0.y * T[1][2][l]
	      + T[2][2][l]);
  
  double v22 =
    (  p0.x * T[0][2][l]
       + p0.y * T[1][2][l]
       + T[2][2][l])
    - 
    (  p0.x * T[0][2][l]
       + p0.y * T[1][2][l]
       + T[2][2][l]);
  
  double v = 0;
  if (fabs(v00) > fabs(v)) v = v00;
  if (fabs(v01) > fabs(v)) v = v01;
  if (fabs(v02) > fabs(v)) v = v02;
  if (fabs(v10) > fabs(v)) v = v10;
  if (fabs(v11) > fabs(v)) v = v11;
  if (fabs(v12) > fabs(v)) v = v12;
  if (fabs(v20) > fabs(v)) v = v20;
  if (fabs(v21) > fabs(v)) v = v21;
  if (fabs(v22) > fabs(v)) v = v22;

  return v;
}

void TrifocalTensor::project(const CvPoint2D64f &p0, 
			     const CvPoint2D64f &p1, 
			     CvPoint2D64f &p2) {
  double z = projectAxis(p0, p1, 2);
  p2.x = projectAxis(p0, p1, 0) / z;
  p2.y = projectAxis(p0, p1, 1) / z;
}

double TrifocalTensor::projectError(const CvPoint2D64f &p0, 
				    const CvPoint2D64f &p1, 
				    const CvPoint2D64f &p2) {
  double v0 = projectAxis(p0, p1, 0);
  double v1 = projectAxis(p0, p1, 1);
  double v2 = projectAxis(p0, p1, 2);

  double e0 = v0/v2 - p2.x;
  double e1 = v1/v2 - p2.y;
  return e0*e0+e1*e1;
}

} // namespace alvar

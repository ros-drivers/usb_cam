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

#include "Pose.h"

using namespace std;

namespace alvar {
using namespace std;

void Pose::Output() const {
	cout<<quaternion[0]<<","<<quaternion[1]<<","<<quaternion[2]<<","<<quaternion[3]<<"|";
	cout<<translation[0]<<","<<translation[1]<<","<<translation[2]<<endl;
}

Pose::Pose() : Rotation() {
	cvInitMatHeader(&translation_mat, 4, 1, CV_64F, translation);
	cvZero(&translation_mat);
	cvmSet(&translation_mat, 3, 0, 1);
}

Pose::Pose(CvMat *tra, CvMat *rot, RotationType t) : Rotation(rot, t) {
	cvInitMatHeader(&translation_mat, 4, 1, CV_64F, translation);
	cvZero(&translation_mat);
	cvmSet(&translation_mat, 3, 0, 1);
	// Fill in translation part
	cvmSet(&translation_mat, 0, 0, cvmGet(tra, 0, 0));
	cvmSet(&translation_mat, 1, 0, cvmGet(tra, 1, 0));
	cvmSet(&translation_mat, 2, 0, cvmGet(tra, 2, 0));
}

Pose::Pose(CvMat *mat) : Rotation(mat, MAT) {
	cvInitMatHeader(&translation_mat, 4, 1, CV_64F, translation);
	cvZero(&translation_mat);
	cvmSet(&translation_mat, 3, 0, 1);
	// Fill in translation part
	if (mat->cols == 4) {
		cvmSet(&translation_mat, 0, 0, cvmGet(mat, 0, 3));
		cvmSet(&translation_mat, 1, 0, cvmGet(mat, 1, 3));
		cvmSet(&translation_mat, 2, 0, cvmGet(mat, 2, 3));
	}
}

Pose::Pose(const Pose& p) :Rotation(p) {
	cvInitMatHeader(&translation_mat, 4, 1, CV_64F, translation);
	cvCopy(&p.translation_mat, &translation_mat);
}

void Pose::Reset()
{
	cvZero(&quaternion_mat); cvmSet(&quaternion_mat, 0, 0, 1);
	cvZero(&translation_mat);
}

void Pose::SetMatrix(const CvMat *mat)
{
	double tmp[9];
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			tmp[i*3+j] = cvmGet(mat, i, j);
			
	Mat9ToQuat(tmp, quaternion);
	if (mat->cols == 4) {
		cvmSet(&translation_mat, 0, 0, cvmGet(mat, 0, 3));
		cvmSet(&translation_mat, 1, 0, cvmGet(mat, 1, 3));
		cvmSet(&translation_mat, 2, 0, cvmGet(mat, 2, 3));
		cvmSet(&translation_mat, 3, 0, 1);
	}
}

void Pose::GetMatrix(CvMat *mat) const
{
	if (mat->width == 3) {
		QuatToMat9(quaternion, mat->data.db);
	} else if (mat->width == 4) {
		cvSetIdentity(mat);
		QuatToMat16(quaternion, mat->data.db);
		cvmSet(mat, 0, 3, cvmGet(&translation_mat, 0, 0));
		cvmSet(mat, 1, 3, cvmGet(&translation_mat, 1, 0));
		cvmSet(mat, 2, 3, cvmGet(&translation_mat, 2, 0));
	}
}

void Pose::GetMatrixGL(double gl[16], bool mirror)
{
	if (mirror) Mirror(false, true, true);
	CvMat gl_mat = cvMat(4, 4, CV_64F, gl);
	GetMatrix(&gl_mat);
	cvTranspose(&gl_mat, &gl_mat);
	if (mirror) Mirror(false, true, true);
}

void Pose::SetMatrixGL(double gl[16], bool mirror)
{
	double gll[16];
	memcpy(gll, gl, sizeof(double)*16);
	CvMat gl_mat = cvMat(4, 4, CV_64F, gll);
	cvTranspose(&gl_mat, &gl_mat);
	SetMatrix(&gl_mat);
	if (mirror) Mirror(false, true, true);
}

void Pose::Transpose()
{
	double tmp[16];
	CvMat tmp_mat = cvMat(4, 4, CV_64F, tmp);
	GetMatrix(&tmp_mat);
	cvTranspose(&tmp_mat, &tmp_mat);
	SetMatrix(&tmp_mat);
}

void Pose::Invert()
{
	double tmp[16];
	CvMat tmp_mat = cvMat(4, 4, CV_64F, tmp);
	GetMatrix(&tmp_mat);
	cvInvert(&tmp_mat, &tmp_mat);
	SetMatrix(&tmp_mat);
}

void Pose::Mirror(bool x, bool y, bool z)
{
	double tmp[16];
	CvMat tmp_mat = cvMat(4, 4, CV_64F, tmp);
	GetMatrix(&tmp_mat);
	MirrorMat(&tmp_mat, x, y, z);
	SetMatrix(&tmp_mat);
}

void Pose::SetTranslation(const CvMat *tra) {
	cvmSet(&translation_mat, 0, 0, cvmGet(tra, 0, 0));
	cvmSet(&translation_mat, 1, 0, cvmGet(tra, 1, 0));
	cvmSet(&translation_mat, 2, 0, cvmGet(tra, 2, 0));
	cvmSet(&translation_mat, 3, 0, 1);
}
void Pose::SetTranslation(const double *tra) {
	translation[0] = tra[0];
	translation[1] = tra[1];
	translation[2] = tra[2];
	translation[3] = 1;
}
void Pose::SetTranslation(const double x, const double y, const double z) {
	translation[0] = x;
	translation[1] = y;
	translation[2] = z;
	translation[3] = 1;
}
void Pose::GetTranslation( CvMat *tra) const{
	cvmSet(tra, 0, 0, cvmGet(&translation_mat, 0, 0));
	cvmSet(tra, 1, 0, cvmGet(&translation_mat, 1, 0));
	cvmSet(tra, 2, 0, cvmGet(&translation_mat, 2, 0));
	if (tra->rows == 4)	cvmSet(tra, 3, 0, 1);
}

Pose& Pose::operator = (const Pose& p)
{
	memcpy(quaternion, p.quaternion, 4*sizeof(double));
	memcpy(translation, p.translation, 4*sizeof(double));
	return *this;
}

} // namespace alvar

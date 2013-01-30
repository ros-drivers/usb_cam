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
#include "Rotation.h"

using namespace std;

namespace alvar {
using namespace std;

Rotation::Rotation()
{
	cvInitMatHeader(&quaternion_mat, 4, 1, CV_64F, quaternion);
	Reset();
}

Rotation::Rotation(const Rotation& r) {
	cvInitMatHeader(&quaternion_mat, 4, 1, CV_64F, quaternion);
	cvCopy(&r.quaternion_mat, &quaternion_mat);
}

Rotation::Rotation(CvMat *data, RotationType t)
{
	cvInitMatHeader(&quaternion_mat, 4, 1, CV_64F, quaternion);

	Reset();

	switch (t)
	{
		case QUAT :
			SetQuaternion(data);
			break;
		case MAT :
			SetMatrix(data);
			break;
		case EUL :
			SetEuler(data);
			break;
		case ROD :
			SetRodriques(data);
			break;
	}
}

void Rotation::Transpose()
{
	double tmp[9];
	CvMat tmp_mat = cvMat(3, 3, CV_64F, tmp);
	GetMatrix(&tmp_mat);
	cvTranspose(&tmp_mat, &tmp_mat);
	SetMatrix(&tmp_mat);
}
	
void Rotation::MirrorMat(CvMat *mat, bool x, bool y, bool z) {
	CvMat *mat_mul = cvCloneMat(mat);
	cvSetIdentity(mat_mul);
	if (x) cvmSet(mat_mul, 0, 0, -1);
	if (y) cvmSet(mat_mul, 1, 1, -1);
	if (z) cvmSet(mat_mul, 2, 2, -1);
	cvMatMul(mat_mul, mat, mat);
	cvReleaseMat(&mat_mul);
}
	
void Rotation::Mirror(bool x, bool y, bool z)
{
	double tmp[9];
	CvMat tmp_mat = cvMat(3, 3, CV_64F, tmp);
	GetMatrix(&tmp_mat);
	MirrorMat(&tmp_mat, x, y, z);
	SetMatrix(&tmp_mat);
}

void Rotation::Reset()
{
	cvZero(&quaternion_mat); cvmSet(&quaternion_mat, 0, 0, 1);
}

void Rotation::Mat9ToRod(double *mat, double *rod)
{
	CvMat mat_m, rod_m;
	cvInitMatHeader(&mat_m, 3, 3, CV_64F, mat);
	cvInitMatHeader(&rod_m, 3, 1, CV_64F, rod);
	cvRodrigues2(&mat_m, &rod_m);
}

void Rotation::RodToMat9(double *rod, double *mat)
{
	CvMat mat_m, rod_m;
	cvInitMatHeader(&mat_m, 3, 3, CV_64F, mat);
	cvInitMatHeader(&rod_m, 3, 1, CV_64F, rod);
	cvRodrigues2(&rod_m, &mat_m, 0);
}

void Rotation::QuatInv(const double *q, double *qi)
{
	qi[0] =    q[0];
	qi[1] = -1*q[1];
	qi[2] = -1*q[2];
	qi[3] = -1*q[3];
}

void Rotation::QuatNorm(double *q)
{
	double l = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
	
	if(l != 0)
		for(unsigned i = 0; i < 4; ++i)
			q[i] = q[i] / l;
}

void Rotation::QuatMul(const double *q1, const double *q2, double *q3)
{
	double w1 = q1[0];
	double x1 = q1[1];
	double y1 = q1[2];
	double z1 = q1[3];

	double w2 = q2[0];
	double x2 = q2[1];
	double y2 = q2[2];
	double z2 = q2[3];

	q3[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
	q3[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
	q3[2] = w1*y2 + y1*w2 + z1*x2 - x1*z2;
	q3[3] = w1*z2 + z1*w2 + x1*y2 - y1*x2;

	QuatNorm(q3);
}

void Rotation::QuatToMat9(const double *quat, double *mat)
{
	double W = quat[0];
	double X = quat[1];
	double Y = quat[2];
	double Z = quat[3];

	double xx = X * X;
	double xy = X * Y;
	double xz = X * Z;
	double xw = X * W;

	double yy = Y * Y;
	double yz = Y * Z;
	double yw = Y * W;

	double zz = Z * Z;
	double zw = Z * W;

	mat[0] = 1 - 2 * ( yy + zz ); //(0,0)
	mat[1] =     2 * ( xy - zw ); //(0,1)
	mat[2] =     2 * ( xz + yw ); //(0,2)

	mat[3] =     2 * ( xy + zw ); //(1,0)
	mat[4] = 1 - 2 * ( xx + zz ); //(1,1)
	mat[5] =     2 * ( yz - xw ); //(1,2)

	mat[6] =     2 * ( xz - yw ); //(2,0)
	mat[7] =     2 * ( yz + xw ); //(2,1)
	mat[8] = 1 - 2 * ( xx + yy ); //(2,2)
}

// TODO: Now we don't want to eliminate the translation part from the matrix here. Did this change break something???
void Rotation::QuatToMat16(const double *quat, double *mat)
{
	//memset(mat, 0, 16*sizeof(double));

	double W = quat[0];
	double X = quat[1];
	double Y = quat[2];
	double Z = quat[3];

	double xx = X * X;
	double xy = X * Y;
	double xz = X * Z;
	double xw = X * W;

	double yy = Y * Y;
	double yz = Y * Z;
	double yw = Y * W;

	double zz = Z * Z;
	double zw = Z * W;

	mat[0] = 1 - 2 * ( yy + zz ); //(0,0)
	mat[1] =     2 * ( xy - zw ); //(0,1)
	mat[2] =     2 * ( xz + yw ); //(0,2)

	mat[4] =     2 * ( xy + zw ); //(1,0)
	mat[5] = 1 - 2 * ( xx + zz ); //(1,1)
	mat[6] =     2 * ( yz - xw ); //(1,2)

	mat[8] =     2 * ( xz - yw ); //(2,0)
	mat[9] =     2 * ( yz + xw ); //(2,1)
	mat[10] = 1 - 2 * ( xx + yy ); //(2,2)

	//mat[15] = 1;
}

void Rotation::QuatToEul(const double *q, double *eul)
{
	double qw = q[0];
	double qx = q[1];
	double qy = q[2];
	double qz = q[3];

	double heading = 0, bank = 0, attitude = 0;

	if ((2*qx*qy + 2*qz*qw) == 1.0)
	{
		heading = 2 * atan2(qx,qw);
		bank = 0;
	}
	else if ((2*qx*qy + 2*qz*qw) == -1.0)
	{
		heading = -2 * atan2(qx,qw);
		bank = 0;
	}
	else
	{
		heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy*qy - 2*qz*qz);
		bank    = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz);
	}

	attitude = asin(2*qx*qy + 2*qz*qw);

	heading  = 180.0 * heading  / PI;
	attitude = 180.0 * attitude / PI;
	bank     = 180.0 * bank     / PI;

	eul[0] = heading;
	eul[1] = attitude;
	eul[2] = bank;
}

void Rotation::Mat9ToQuat(const double *mat, double *quat)
{
	quat[0] = sqrt(max(0., 1 + mat[0] + mat[4] + mat[8])) / 2.0;  // w
	quat[1] = sqrt(max(0., 1 + mat[0] - mat[4] - mat[8])) / 2.0;  // x
	quat[2] = sqrt(max(0., 1 - mat[0] + mat[4] - mat[8])) / 2.0;  // y
	quat[3] = sqrt(max(0., 1 - mat[0] - mat[4] + mat[8])) / 2.0;  // z

	quat[1] = quat[1]*Sign(mat[7] - mat[5]); // x
	quat[2] = quat[2]*Sign(mat[2] - mat[6]); // y
	quat[3] = quat[3]*Sign(mat[3] - mat[1]); // z

	QuatNorm(quat);
}

void Rotation::EulToQuat(const double *eul, double *quat)
{
	double heading  = PI*eul[0]/180.0;
	double attitude = PI*eul[1]/180.0;
	double bank     = PI*eul[2]/180.0;

	double c1 = cos(heading/2.0);
	double s1 = sin(heading/2.0);
	double c2 = cos(attitude/2.0);
	double s2 = sin(attitude/2.0);
	double c3 = cos(bank/2.0);
	double s3 = sin(bank/2.0);
	double c1c2 = c1*c2;
	double s1s2 = s1*s2;

	quat[0] = c1c2*c3  - s1s2*s3;
	quat[1] = c1c2*s3  + s1s2*c3;
	quat[2] = s1*c2*c3 + c1*s2*s3;
	quat[3] = c1*s2*c3 - s1*c2*s3;

	QuatNorm(quat);
}

void Rotation::SetQuaternion(CvMat *mat)
{
	cvCopy(mat, &quaternion_mat);
	QuatNorm(quaternion);
}

void Rotation::SetQuaternion(const double *quat)
{
	quaternion[0] = quat[0];
	quaternion[1] = quat[1];
	quaternion[2] = quat[2];
	quaternion[3] = quat[3];
	QuatNorm(quaternion);
}

void Rotation::SetEuler(const CvMat *mat)
{
	EulToQuat(mat->data.db, quaternion);
}

void Rotation::SetRodriques(const CvMat *mat)
{
	double tmp[9];
	RodToMat9(mat->data.db, tmp);
	Mat9ToQuat(tmp, quaternion);
}

void Rotation::SetMatrix(const CvMat *mat)
{
	double tmp[9];
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			tmp[i*3+j] = cvmGet(mat, i, j);

	Mat9ToQuat(tmp, quaternion);
}

void Rotation::GetMatrix(CvMat *mat) const
{
	if (mat->width == 3) {
		QuatToMat9(quaternion, mat->data.db);
	} else if (mat->width == 4) {
		cvSetIdentity(mat);
		QuatToMat16(quaternion, mat->data.db);
	}
}

void Rotation::GetRodriques(CvMat *mat) const
{
	double tmp[9];
	QuatToMat9(quaternion, tmp);
	Mat9ToRod(tmp, mat->data.db);
}

void Rotation::GetEuler(CvMat *mat) const
{
	QuatToEul(quaternion, mat->data.db);
}

void Rotation::GetQuaternion(CvMat *mat) const
{
	cvCopy(&quaternion_mat, mat);
}

// TODO: This is not needed???
inline Rotation& Rotation::operator = (const Rotation& r)
{
	memcpy(quaternion, r.quaternion, 4*sizeof(double));
	return *this;
}

inline Rotation& Rotation::operator += (const Rotation& r)
{
	Rotation res;
	QuatMul(quaternion, r.quaternion, res.quaternion);
	memcpy(quaternion, res.quaternion, 4*sizeof(double));
	//x += v.x;
	//y += v.y;
	//z += v.z;

	return *this;
}

inline Rotation operator + (const Rotation& r1, const Rotation& r2)
{
	Rotation ret = r1;
	ret += r2;

	return ret;
}

} // namespace alvar

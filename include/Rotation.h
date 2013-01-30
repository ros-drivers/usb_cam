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

#ifndef ROTATION_H
#define ROTATION_H

#include <iostream>

/**
 * \file Rotation.h
 *
 * \brief This file implements a parametrized rotation.
 */

#include "Alvar.h"
#include "Util.h"

namespace alvar {

/**
 * \brief \e Rotation structure and transformations between different parameterizations.
 */
class ALVAR_EXPORT Rotation
{

public:	
	CvMat quaternion_mat;
	double quaternion[4];
	
	/**
	* \brief Rotation can be represented in four ways: quaternion (QUAT), matrix (MAT), euler angles (EUL) and exponential map (ROD).
	*/
	enum RotationType {QUAT, MAT, EUL, ROD};

	Rotation();
	Rotation(const Rotation& r);
	
	/**
	* \brief Constructor.
	* \param data	Rotation data stored in CvMat. With RotationType::MAT both 3x3 and 4x4 matrices are allowed.
	* \param t		Rotation type that corresponds to data.
	*/
	Rotation(CvMat *data, RotationType t);

	Rotation& operator  = (const Rotation& p);
	Rotation& operator += (const Rotation& v);
	//Rotation& operator +  () {return *this;}

	void Transpose();
	
	/**
	* \brief Simple function to mirror a rotation matrix in different directions.
	* \param mat	Matrix to be mirrored.
	* \param x		
	* \param y		
	* \param z		
	*/
	static void MirrorMat(CvMat *mat, bool x, bool y, bool z);
	
	/**
	* \brief Mirrors the rotation in selected directions.
	* \param x	
	* \param y	
	* \param z	
	*/
	void Mirror(bool x, bool y, bool z);

	/**
	* \brief Resets the rotation into identity.
	*/
	void Reset();

	/**
	* \brief Converts 3x3 rotation matrix into Rodriques representation.
	* \param mat	3x3 rotation matrix.
	* \param rod	Resulting 3x1 rotation vector.
	*/
	static void Mat9ToRod(double *mat, double *rod);

	/**
	* \brief Converts 3x1 rotation vector into 3x3 rotation matrix using Rodriques' formula.
	* \param rod 3x1 rotation vector.
	* \param Resulting 3x3 rotation matrix.
	*/
	static void RodToMat9(double *rod, double *mat);

	/**
	* \brief Inverts unit quaternion.
	* \param q	Unit quaternion to be inverted.
	* \param qi	Resulting quaternion.
	*/
	static void QuatInv(const double *q, double *qi);
	
	/**
	* \brief Normalizes a quaternion.
	* \param q	Quaternion to be normalized.
	*/
	static void QuatNorm(double *q);
	
	/**
	* \brief Quaternion multiplication.
	* \param q1	
	* \param q2	
	* \param q3	Resulting quaternion.
	*/
	static void QuatMul(const double *q1, const double *q2, double *q3);

	//%  The quaternion has to be normalized!!!
	/**
	* \brief Converts a rotation described by a quaternion into 3x3 rotation matrix.
	* \param quat	Rotation in quaternion form.
	* \param mat	Corresponding 3x3 rotation matrix.
	*/
	static void QuatToMat9(const double *quat, double *mat);

	// TODO: Now we don't want to eliminate the translation part from the matrix here. Did this change break something???
	/**
	* \brief Converts a rotation described by a quaternion into 4x4 OpenGL-like transformation matrix. The translation part is not altered.
	* \param quat	Rotation in quaternion form.
	* \param mat	Resulting 4x4 transformation matrix.
	*/
	static void QuatToMat16(const double *quat, double *mat);
	
	/**
	* \brief Converts a rotation described by a quaternion into Euler angles.
	* \param q		Rotation in quaternion form.
	* \param eul	Resulting Euler angles.
	*/
	static void QuatToEul(const double *q, double *eul);
	
	/**
	* \brief Converts a 3x3 rotation martix into quaternion form.
	* \param mat	3x3 rotation matrix.
	* \param quat	Resulting quaternion.
	*/
	static void Mat9ToQuat(const double *mat, double *quat);
	
	/**
	* \brief Converts a rotation described by Euler angles into quaternion form.
	* \param eul	Rotation in Euler angles.
	* \param quat	Resulting quaternion.
	*/
	static void EulToQuat(const double *eul, double *quat);
	
	/**
	* \brief Sets the rotation from given quaternion. 
	* \param mat	Input quaternion (4x1 CvMat).
	*/
	void SetQuaternion(CvMat *mat);
	
	/**
	* \brief Sets the rotation from given quaternion. 
	* \param mat	Input quaternion (4x1 double array).
	*/
	void SetQuaternion(const double *quat);
	
	/**
	* \brief Sets the rotation from given Euler angles.
	* \param mat	Input Euler angles (3x1 CvMat).
	*/
	void SetEuler(const CvMat *mat);
	
	/**
	* \brief Sets the rotation from given rotation vector.
	* \param mat	Input rotation vector (3x1 CvMat).
	*/
	void SetRodriques(const CvMat *mat);
	
	/**
	* \brief Sets the rotation from given rotation matrix. 3x3 and 4x4 matrices are allowed.
	* \param mat	Input rotation matrix (3x3 or 4x4 CvMat).
	*/
	void SetMatrix(const CvMat *mat);
	
	/**
	* \brief Returns the rotation in matrix form. 3x3 and 4x4 matrices are allowed.
	* \param mat	The rotation is stored here.
	*/
	void GetMatrix(CvMat *mat) const;
	
	/**
	* \brief Returns the rotation in rotation vector form.
	* \param mat	The rotation is stored here (3x1 CvMat).
	*/
	void GetRodriques(CvMat *mat) const;
	
	/**
	* \brief Returns the rotation in Euler angles.
	* \param mat	The rotation is stored here (3x1 CvMat).
	*/
	void GetEuler(CvMat *mat) const;
	
	/**
	* \brief Returns the rotation in quaternion form.
	* \param mat	The rotation is stored here (4x1 CvMat).
	*/
	void GetQuaternion(CvMat *mat) const;
};

} // namespace alvar

#endif

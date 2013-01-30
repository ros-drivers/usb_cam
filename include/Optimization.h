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

#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "Alvar.h"
#include <cxcore.h>
//#include <float.h>


/**
 * \file Optimization.h
 *
 * \brief This file implements several optimization algorithms.
 */

namespace alvar {

/** 
  * \brief Non-linear optimization routines. There are three methods implemented that include Gauss-Newton, Levenberg-Marquardt and Tukey m-estimator.
  *  
  */
class ALVAR_EXPORT Optimization 
{

private:

	void *estimate_param;
	CvMat *J;
	CvMat *JtJ;
	CvMat *W;
	CvMat *diag;
	CvMat *tmp;
	CvMat *err;
	CvMat *delta;
	CvMat *x_plus;
	CvMat *x_minus;
	CvMat *x_tmp1;
	CvMat *x_tmp2;
	CvMat *tmp_par;

	double CalcTukeyWeight(double residual, double c);
	double CalcTukeyWeightSimple(double residual, double c);

	double lambda;

public:

	/**
	  * \brief Selection between the algorithm used in optimization. Following should be noticed:
	  * \li GAUSSNEWTON
	  */
	enum OptimizeMethod
	{
		GAUSSNEWTON,
		LEVENBERGMARQUARDT,
		TUKEY_LM
	};

	/**
      * \brief Constructor.
	  * \param n_params	Number of parameters to be optimized.
      * \param n_meas	Number of measurements that are observed.
	  */
	Optimization(int n_params, int n_meas);
	~Optimization();

	/**
	  * \brief Returns the current residual vector.
	  * \return Pointer to the residual vector.
	  */
	CvMat *GetErr() { return err; }

	/**
	  * \brief Pointer to the function that projects the state of the system to the measurements.
	  * \param state		System parameters, e.g. camera parameterization in optical tracking.
	  * \param projection	The system state projection is stored here. E.g image measurements in optical tracking.
	  * \param param		Additional parameters to the function. E.g. some constant parameters that are not optimized.
	  */
	typedef void (*EstimateCallback)(CvMat* state, CvMat *projection, void *param);

	/** 
	  * \brief Numerically differentiates and calculates the Jacobian around x.
	  * \param x		The set of parameters around which the Jacobian is evaluated.
	  * \param J		Resulting Jacobian matrix is stored here.
	  * \param Estimate	The function to be differentiated.
	  */
	void CalcJacobian(CvMat* x, CvMat* J, EstimateCallback Estimate);

	/**
	  * \brief Runs the optimization loop with selected parameters.
	  * \param parameters		Vector of parameters to be optimized. Initial values should be set.
	  * \param measurements		Vector of measurements that are observed. 
	  * \param stop				Optimization loop ends as the \e stop limit is reached. Criteria is calculated as 
	  * \param max_iter			Maximum number of iteration loops that are evaluated if \e stop is not reached.
	  * \param Estimate			Pointer to the function that maps the state to the measurements. See \e EstimateCallback.
	  * \param method			One of the three possible optimization methods.
	  * \param parameters_mask	Vector that defines the parameters that are optimized. If vector element is 0, corresponding parameter is not altered.
	  * \param J_mat			Jacobian matrix. If not given, numerical differentation is used.
	  * \param weights			Weight vector that can be submitted to give different weights to different measurements. Currently works only with OptimizeMethod::TUKEY_LM.
	  */
	double Optimize(CvMat*					parameters,
				    CvMat*					measurements,
					double					stop,
					int						max_iter,
					EstimateCallback		Estimate,
					void *param				= 0,
					OptimizeMethod method	= LEVENBERGMARQUARDT,
					CvMat* parameters_mask	= 0,
					CvMat* J_mat			= 0,
					CvMat* weights			= 0); 

};

} // namespace alvar

#endif

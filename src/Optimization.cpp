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
#include "Optimization.h"
#include "time.h"
#include "highgui.h"

#include <iostream>
using namespace std;

namespace alvar {

Optimization::Optimization(int n_params, int n_meas)
{
	estimate_param = 0;
	J       = cvCreateMat(n_meas,   n_params, CV_64F); cvZero(J);
	JtJ     = cvCreateMat(n_params, n_params, CV_64F); cvZero(JtJ);
	tmp     = cvCreateMat(n_params, n_meas,   CV_64F); cvZero(tmp);
	W       = cvCreateMat(n_meas,   n_meas,   CV_64F); cvZero(W); 
	diag    = cvCreateMat(n_params, n_params, CV_64F); cvZero(diag);
	err     = cvCreateMat(n_meas,   1, CV_64F); cvZero(err);
	delta   = cvCreateMat(n_params, 1, CV_64F); cvZero(delta);
	x_minus = cvCreateMat(n_params, 1, CV_64F); cvZero(x_minus);
	x_plus  = cvCreateMat(n_params, 1, CV_64F); cvZero(x_plus);
	x_tmp1  = cvCreateMat(n_meas,   1, CV_64F); cvZero(x_tmp1);
	x_tmp2  = cvCreateMat(n_meas,   1, CV_64F); cvZero(x_tmp2);
	tmp_par = cvCreateMat(n_params, 1, CV_64F); cvZero(tmp_par);
}

Optimization::~Optimization()
{
	cvReleaseMat(&J);
	cvReleaseMat(&JtJ);
	cvReleaseMat(&diag);
	cvReleaseMat(&tmp);
	cvReleaseMat(&W);
	cvReleaseMat(&err);
	cvReleaseMat(&delta);
	cvReleaseMat(&x_plus);
	cvReleaseMat(&x_minus);
	cvReleaseMat(&x_tmp1);
	cvReleaseMat(&x_tmp2);
	cvReleaseMat(&tmp_par);
	estimate_param = 0;
}

double Optimization::CalcTukeyWeight(double residual, double c)
{
	//const double c = 3; // squared distance in the model tracker
	double ret=0;

	if(fabs(residual) <= c)
	{
		double tmp = 1.0-((residual/c)*(residual/c));
		ret = ((c*c)/6.0)*(1.0-tmp*tmp*tmp);
	}
	else
		ret = (c*c)/6.0;
	
	if(residual)
		ret = fabs(sqrt(ret)/residual);
	else
		ret = 1.0; // ???
	
	return ret;
}

double Optimization::CalcTukeyWeightSimple(double residual, double c)
{
	//const double c = 3;
	double ret=0;

	double x2 = residual*residual;
	if(x2<c*c) return residual;
	else return c;
}

void Optimization::CalcJacobian(CvMat* x, CvMat* J, EstimateCallback Estimate)
{
	const double step = 0.001;

	cvZero(J);
	for (int i=0; i<J->cols; i++)
	{
		CvMat J_column;
		cvGetCol(J, &J_column, i);

		cvZero(delta); 
		cvmSet(delta, i, 0, step);
		cvAdd(x, delta, x_plus);
		cvmSet(delta, i, 0, -step);
		cvAdd(x, delta, x_minus);

		Estimate(x_plus,  x_tmp1, estimate_param);
		Estimate(x_minus, x_tmp2, estimate_param);
		cvSub(x_tmp1, x_tmp2, &J_column);
		cvScale(&J_column, &J_column, 1.0/(2*step));
	}
}


double Optimization::Optimize(CvMat* parameters,      // Initial values are set
							  CvMat* measurements,    // Some observations
							  double stop,
							  int	max_iter,
							  EstimateCallback Estimate,
							  void *param,
							  OptimizeMethod method,
							  CvMat* parameters_mask, // Mask indicating non-constant parameters)
							  CvMat* J_mat,
							  CvMat* weights)
{

	int n_params = parameters->rows;
	int n_meas   = measurements->rows;
	double error_new = 0;
	double error_old = 0;
	double n1, n2;
	int cntr = 0;
	estimate_param = param;
	lambda = 0.001;

	while(true)
	{
		if(!J_mat)
			CalcJacobian(parameters, J, Estimate);
		else
			J = J_mat;

		// Zero the columns for constant parameters
		// TODO: Make this into a J-sized mask matrix before the iteration loop
		if(parameters_mask)
		for (int i=0; i<parameters_mask->rows; i++) {
			if (cvGet2D(parameters_mask, i, 0).val[0] == 0) {
				CvRect rect;
				rect.height = J->rows; rect.width = 1;
				rect.y = 0; rect.x = i;
				CvMat foo;
				cvGetSubRect(J, &foo, rect);
				cvZero(&foo);
			}
		}

		Estimate(parameters, x_tmp1, estimate_param);
		cvSub(measurements, x_tmp1, err); // err = residual
		error_old = cvNorm(err, 0, CV_L2);

		switch(method)
		{
			case (GAUSSNEWTON) :

				cvMulTransposed(J, JtJ, 1);
				cvInv(JtJ, JtJ, CV_SVD);
				cvGEMM(JtJ, J, 1.0, 0, 0, tmp, CV_GEMM_B_T); // inv(JtJ)Jt
				cvMatMul(tmp, err, delta);
				cvAdd(delta, parameters, parameters);

				// Lopetusehto
				n1 = cvNorm(delta);
				n2 = cvNorm(parameters);

				if( ((n1/n2) < stop) ||
					(cntr >= max_iter) )
					goto end; 

			break;

			case (LEVENBERGMARQUARDT) :

				cvSetIdentity(diag, cvRealScalar(lambda));

				if(weights)
					for(int k = 0; k < W->rows; ++k)
						cvmSet(W, k, k, weights->data.db[k]);

				// JtWJ
				if(weights)
				{
					cvGEMM(J, W, 1, 0, 0, tmp, CV_GEMM_A_T);
					cvGEMM(tmp, J, 1, 0, 0, JtJ, 0);
				}
				else
					cvMulTransposed(J, JtJ, 1);

				// JtJ + lambda*I
				// or JtWJ + lambda*I if weights are used...
				cvAdd(JtJ, diag, JtJ);
				cvInv(JtJ, JtJ, CV_SVD);
				cvGEMM(JtJ, J, 1.0, 0, 0, tmp, CV_GEMM_B_T);
				
				if(weights)
					cvGEMM(tmp, W, 1, 0, 0, tmp, 0);
				
				cvMatMul(tmp, err, delta);
				cvAdd(delta, parameters, tmp_par);

				Estimate(tmp_par, x_tmp1, estimate_param);
				cvSub(measurements, x_tmp1, err);

				error_new = cvNorm(err, 0, CV_L2);
			
				if(error_new < error_old)
				{
					cvCopy(tmp_par, parameters);
					lambda = lambda/10.0;
				}
				else
				{
					lambda = lambda*10.0;
				}
				if(lambda>10) lambda = 10;
				if(lambda<0.00001) lambda = 0.00001;

				n1 = cvNorm(delta);
				n2 = cvNorm(parameters);

				if( (n1/n2) < stop   ||
					(cntr >= max_iter) )
				{
					goto end;
				}

			break;

			case (TUKEY_LM) :
							
				cvSetIdentity(diag, cvRealScalar(lambda));

				// Tukey weights
				for(int k = 0; k < W->rows; ++k)
				{
					if(weights)													  // If using weight vector
						if(weights->data.db[k] != -1.0)							     // If true weight given
							cvmSet(W, k, k, weights->data.db[k]);				          // Use given weight
						else
							cvmSet(W, k, k, CalcTukeyWeight(err->data.db[k], 3));     // otherwise use Tukey weight
					else
						cvmSet(W, k, k, CalcTukeyWeight(err->data.db[k], 3));	  // Otherwise use Tukey weight
				}

				cvGEMM(J, W, 1, 0, 0, tmp, CV_GEMM_A_T);
				cvGEMM(tmp, J, 1, 0, 0, JtJ, 0);
				cvAdd(JtJ, diag, JtJ);
				cvInv(JtJ, JtJ, CV_SVD);
				cvGEMM(JtJ, J, 1.0, 0, 0, tmp, CV_GEMM_B_T);
				cvGEMM(tmp, W, 1, 0, 0, tmp, 0);
				cvMatMul(tmp, err, delta);
				cvAdd(delta, parameters, tmp_par);

				Estimate(tmp_par, x_tmp1, estimate_param);
				cvSub(measurements, x_tmp1, err);

				error_new = cvNorm(err, 0, CV_L2);
				
				if(error_new < error_old)
				{
					cvCopy(tmp_par, parameters);
					lambda = lambda/10.0;
				}
				else
				{
					lambda = lambda*10.0;
				}
				if(lambda>10) lambda = 10;
				if(lambda<0.00001) lambda = 0.00001;

				n1 = cvNorm(delta);
				n2 = cvNorm(parameters);

				if( ((n1/n2) < stop) ||
					(cntr >= max_iter) )
				{
					goto end;
				}
	
			break;
		}
		++cntr;
	}

end :

	return error_old;
}

} // namespace alvar

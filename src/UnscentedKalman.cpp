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

#include "UnscentedKalman.h"
#include <cxcore.h>
#include <stdio.h>

namespace alvar {

UnscentedKalman::UnscentedKalman(int state_n, int obs_n, int state_k, double alpha, double beta) {
  state_k = 0;
  //TODO: support a separate noise vector/covariance matrix: state_k;
  this->state_k = state_k;
  this->state_n = state_n;
  this->obs_n = obs_n;
  sigma_n = 2 * state_n + 1;

  double L = state_n + state_k;
  lambda = alpha*alpha * L - L;
  lambda2 = 1 - alpha*alpha + beta;

  state = cvCreateMat(state_n, 1, CV_64F); cvSetZero(state);
  stateCovariance = cvCreateMat(state_n, state_n, CV_64F); cvSetZero(stateCovariance);
  sqrtStateCovariance = cvCreateMat(state_n, state_n, CV_64F); cvSetZero(sqrtStateCovariance);
  stateD = cvCreateMat(state_n, state_n, CV_64F); cvSetZero(stateD);
  stateU = cvCreateMat(state_n, state_n, CV_64F); cvSetZero(stateU);
  stateV = cvCreateMat(state_n, state_n, CV_64F); cvSetZero(stateV);
  stateTmp = cvCreateMat(state_n, state_n, CV_64F); cvSetZero(stateTmp);
  stateDiff = cvCreateMat(state_n, 1, CV_64F); cvSetZero(stateDiff);

  predObs = cvCreateMat(obs_n, 1, CV_64F); cvSetZero(predObs);
  predObsCovariance = cvCreateMat(obs_n, obs_n, CV_64F); cvSetZero(predObsCovariance);
  predObsDiff = cvCreateMat(obs_n, 1, CV_64F); cvSetZero(predObsDiff);

  invPredObsCovariance = cvCreateMat(obs_n, obs_n, CV_64F); cvSetZero(invPredObsCovariance);
  statePredObsCrossCorrelation = cvCreateMat(state_n, obs_n, CV_64F); cvSetZero(statePredObsCrossCorrelation);
  kalmanGain = cvCreateMat(state_n, obs_n, CV_64F); cvSetZero(kalmanGain);
  kalmanTmp = cvCreateMat(state_n, obs_n, CV_64F); cvSetZero(kalmanTmp);

  sigma_state = new CvMat*[sigma_n];
  sigma_predObs = new CvMat*[sigma_n];

  for (int i = 0; i < sigma_n; i++) {
    sigma_state[i] = cvCreateMat(state_n, 1, CV_64F); cvSetZero(sigma_state[i]);
    sigma_predObs[i] = cvCreateMat(obs_n, 1, CV_64F); cvSetZero(sigma_predObs[i]);
  }

  sigmasUpdated = false;
}

UnscentedKalman::~UnscentedKalman() {
  cvReleaseMat(&state);
  cvReleaseMat(&stateCovariance);
  cvReleaseMat(&sqrtStateCovariance);
  cvReleaseMat(&stateD);
  cvReleaseMat(&stateU);
  cvReleaseMat(&stateV);
  cvReleaseMat(&stateTmp);
  cvReleaseMat(&stateDiff);
  cvReleaseMat(&kalmanTmp);
  cvReleaseMat(&kalmanGain);
  cvReleaseMat(&statePredObsCrossCorrelation);
  cvReleaseMat(&invPredObsCovariance);

  cvReleaseMat(&predObs);
  cvReleaseMat(&predObsCovariance);
  cvReleaseMat(&predObsDiff);

  for (int i = 0; i < sigma_n; i++) {
    cvReleaseMat(&sigma_state[i]);
    cvReleaseMat(&sigma_predObs[i]);
  }
  delete[] sigma_state;
  delete[] sigma_predObs;
}

void UnscentedKalman::initialize() {
  // Computes new sigma points from current state estimate.

  // 1. Compute square root of state co-variance:
  //[E D] = eig(A); sqrtm(A) = E * sqrt(D) * E' where D is a diagonal matrix.
  //sqrt(D) is formed by taking the square root of the diagonal entries in D.
 #ifdef MYDEBUG
	printf("stateCovariance:\n");
	for (int i = 0; i < 5; i++)
	  printf("%+.10f %+.10f %+.10f %+.10f %+.10f\n",
		 cvGetReal2D(stateCovariance, 0, i),
		 cvGetReal2D(stateCovariance, 1, i),
		 cvGetReal2D(stateCovariance, 2, i),
		 cvGetReal2D(stateCovariance, 3, i),
		 cvGetReal2D(stateCovariance, 4, i));
#endif

  //Another equivilant way is to use:
  // [U S V] = svd(A); sqrtm(A) = U * sqrt(S) * V'
  cvSVD(stateCovariance, stateD, stateU, stateV); //, CV_SVD_V_T
  double L = state_n + state_k;
  double scale = L + lambda;
  for (int i = 0; i < state_n; i++) {
    double d = cvGetReal2D(stateD, i, i);
    cvSetReal2D(stateD, i, i, sqrt(scale*d));
  }
  cvGEMM(stateD, stateV, 1., NULL, 0, stateTmp, CV_GEMM_B_T);
  cvGEMM(stateU, stateTmp, 1., NULL, 0, sqrtStateCovariance);
#ifdef MYDEBUG
	printf("sqrtStateCovariance:\n");
	for (int i = 0; i < 5; i++)
	  printf("%+.10f %+.10f %+.10f %+.10f %+.10f\n",
		 cvGetReal2D(sqrtStateCovariance, 0, i),
		 cvGetReal2D(sqrtStateCovariance, 1, i),
		 cvGetReal2D(sqrtStateCovariance, 2, i),
		 cvGetReal2D(sqrtStateCovariance, 3, i),
		 cvGetReal2D(sqrtStateCovariance, 4, i));
	cvGEMM(sqrtStateCovariance, sqrtStateCovariance, 1., NULL, 0, stateTmp);
	printf("sqrtStateCovariance^2:\n");
	for (int i = 0; i < 5; i++)
	  printf("%+.10f %+.10f %+.10f %+.10f %+.10f\n",
		 cvGetReal2D(stateTmp, 0, i),
		 cvGetReal2D(stateTmp, 1, i),
		 cvGetReal2D(stateTmp, 2, i),
		 cvGetReal2D(stateTmp, 3, i),
		 cvGetReal2D(stateTmp, 4, i));
#endif

  // 2. Form new sigma points.
  int sigma_i = 0;
  cvCopy(state, sigma_state[sigma_i++]);
  for (int i = 0; i < state_n; i++) {
    CvMat col;
    cvGetCol(sqrtStateCovariance, &col, i);
    cvAdd(state, &col, sigma_state[sigma_i++]);
    cvSub(state, &col, sigma_state[sigma_i++]);
  }

  sigmasUpdated = true;
}

void UnscentedKalman::predict(UnscentedProcess *process_model) {
  if (!sigmasUpdated) initialize();

  // Map sigma points through the process model and compute new state mean.
  cvSetZero(state);
  double L = state_n + state_k;
  double totalWeight = 0;
  for (int i = 0; i < sigma_n; i++) {
    double weight = i == 0 
      ? lambda / (L + lambda)
      : .5 / (L + lambda);
	totalWeight += weight;
  }
  for (int i = 0; i < sigma_n; i++) {
    CvMat *sigma = sigma_state[i];
    process_model->f(sigma);
    double weight = i == 0 
      ? lambda / (L + lambda)
      : .5 / (L + lambda);
	double scale = weight / totalWeight;
    cvAddWeighted(sigma, scale, state, 1., 0., state);
  }

  // Compute new state co-variance.
  cvSetZero(stateCovariance);
  totalWeight = 0;
  for (int i = 0; i < sigma_n; i++) {
    double weight = i == 0 
      ? lambda / (L + lambda) + lambda2
      : .5 / (L + lambda);
	totalWeight += weight;
  }
  for (int i = 0; i < sigma_n; i++) {
    double weight = i == 0 
      ? lambda / (L + lambda) + lambda2
      : .5 / (L + lambda);
	double scale = weight / totalWeight;
    cvSub(sigma_state[i], state, stateDiff);
    cvGEMM(stateDiff, stateDiff, scale, stateCovariance, 1., stateCovariance,
	   CV_GEMM_B_T);
  }

  // Add any additive noise.
  CvMat *noise = process_model->getProcessNoise();
  if (noise) cvAdd(stateCovariance, noise, stateCovariance);

#ifdef MYDEBUG
  printf("predicted state: ");
  for (int i = 0; i < state_n; i++) printf("%f ", cvGetReal1D(state, i));
  printf("\n");
  printf("predicted stateCovariance:\n");
  for (int i = 0; i < state_n; i++) {
    for (int j = 0; j < state_n; j++) printf("%+f ", cvGetReal2D(stateCovariance, i, j));
    printf("\n");
  }
#endif

  sigmasUpdated = false;
}

void UnscentedKalman::update(UnscentedObservation *obs) {
  if (!sigmasUpdated) initialize();
  CvMat *innovation = obs->getObservation();
  int obs_n = innovation->rows;
  if (obs_n > this->obs_n) {
    printf("Observation exceeds maximum size!\n");
    abort();
  }

  // Map sigma points through the observation model and compute predicted mean.
  CvMat predObs = cvMat(obs_n, 1, CV_64F, this->predObs->data.db);
  cvSetZero(&predObs);
  for (int i = 0; i < sigma_n; i++) {
    CvMat sigma_h = cvMat(obs_n, 1, CV_64F, sigma_predObs[i]->data.db);
    double scale = i == 0 
      ? (double)state_k / (double)(state_n + state_k)
      : .5 / (double)(state_n + state_k);
    obs->h(&sigma_h, sigma_state[i]);
    cvAddWeighted(&sigma_h, scale, &predObs, 1., 0., &predObs);
  }

  // Compute predicted observation co-variance.
  CvMat predObsCovariance = cvMat(obs_n, obs_n, CV_64F, 
                                  this->predObsCovariance->data.db);
  CvMat statePredObsCrossCorrelation = cvMat(state_n, obs_n, CV_64F, 
                                             this->statePredObsCrossCorrelation->data.db);
  CvMat predObsDiff = cvMat(obs_n, 1, CV_64F, this->predObsDiff->data.db);
  cvSetZero(&predObsCovariance);
  cvSetZero(&statePredObsCrossCorrelation);
  for (int i = 0; i < sigma_n; i++) {
    CvMat sigma_h = cvMat(obs_n, 1, CV_64F, sigma_predObs[i]->data.db);
    double scale = i == 0 
      ? (double)state_k / (double)(state_n + state_k)
      : .5 / (double)(state_n + state_k);
    cvSub(sigma_state[i], state, stateDiff);
    cvSub(&sigma_h, &predObs, &predObsDiff);
    cvGEMM(&predObsDiff, &predObsDiff, scale, &predObsCovariance, 1., &predObsCovariance,
	         CV_GEMM_B_T);
    cvGEMM(stateDiff, &predObsDiff, scale, &statePredObsCrossCorrelation, 1., 
	         &statePredObsCrossCorrelation, CV_GEMM_B_T);
  }

  // Add any additive noise.
  CvMat *noise = obs->getObservationNoise();
  if (noise) cvAdd(&predObsCovariance, noise, &predObsCovariance);

#ifdef MYDEBUG
  printf("real observation: ");
  for (int i = 0; i < obs_n; i++) printf("%+f ", cvGetReal1D(innovation ,i));
  printf("\n");
  printf("predicted observation: ");
  for (int i = 0; i < obs_n; i++) printf("%+f ", cvGetReal1D(&predObs,i));
  printf("\n");
  printf("predicted observation co-variance\n");
  for (int i = 0; i < obs_n; i++) {
    for (int j = 0; j < obs_n; j++) printf("%+f ", cvGetReal2D(&predObsCovariance,i,j));
    printf("\n");
  }
  printf("state observation cross-correlation\n");
  for (int i = 0; i < state_n; i++) {
    for (int j = 0; j < obs_n; j++) printf("%+f ", cvGetReal2D(&statePredObsCrossCorrelation,i,j));
    printf("\n");
  }
#endif

  // Update state mean and co-variance.
  //  innovation: v = z - pz
  //  gain: W = XZ * (R + Z)^-1
  //  state: x = x + _W * v
  //  co-var: P = P - W * (R + Z) * W^T

  CvMat invPredObsCovariance = cvMat(obs_n, obs_n, CV_64F, 
                                     this->invPredObsCovariance->data.db);
  CvMat kalmanGain = cvMat(state_n, obs_n, CV_64F, this->kalmanGain->data.db);
  CvMat kalmanTmp = cvMat(state_n, obs_n, CV_64F, this->kalmanTmp->data.db);

  cvSub(innovation, &predObs, innovation);
  //double inno_norm = cvNorm(innovation) / obs_n;
  //if (inno_norm > 5.0) {
  //  return;
  //}

#ifdef MYDEBUG
  printf("innovation: ");
  for (int i = 0; i < obs_n; i++) printf("%f ", cvGetReal1D(innovation,i));
  printf("\n");
  double inn_norm = cvNorm(innovation);
  printf("innivation norm: %f\n", inn_norm);
#endif

  cvInvert(&predObsCovariance, &invPredObsCovariance, CV_SVD_SYM);
  cvMatMul(&statePredObsCrossCorrelation, &invPredObsCovariance, &kalmanGain);
  cvGEMM(&kalmanGain, innovation, 1., state, 1., state);
  cvMatMul(&kalmanGain, &predObsCovariance, &kalmanTmp);
  cvGEMM(&kalmanTmp, &kalmanGain, -1., stateCovariance, 1., stateCovariance,
	       CV_GEMM_B_T);
#ifdef MYDEBUG
  printf("estimated state: ");
  for (int i = 0; i < state_n; i++) printf("%f ", cvGetReal1D(state, i));
  printf("\n");
#endif

  sigmasUpdated = false;
}

} // namespace alvar

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

#ifndef KALMAN_H
#define KALMAN_H

/**
 * \file Kalman.h
 * 
 * \brief This file implements a versatile Kalman filter.
 */

#include "Alvar.h"
#include "cxcore.h"

namespace alvar {

/** \brief Core implementation for Kalman sensor */
class ALVAR_EXPORT KalmanSensorCore {
	friend class KalmanVisualize;
protected:
	int n;
	int m; 
	CvMat *H_trans;
	CvMat *z_pred;
	CvMat *z_residual;
	CvMat *x_gain;
public:
	/** \brief Latest measurement vector (m*1) */
	CvMat *z; 
	/** \brief The matrix (m*n) mapping Kalman state vector into this sensor's measurements vector */
	CvMat *H;
	/** \brief The matrix (n*m) containing Kalman gain (something between 0 and H^-1). 
	* In this core-implementation we assume this to be precalculated. In \e KalmanSensor this is updated using \e update_K .
	*/
	CvMat *K;
	/** \brief Copy constructor */
	KalmanSensorCore(const KalmanSensorCore &k);
	/** 
	* \brief 		Constructor
	* \param _n 	The number of items in the Kalman  state vector
	* \param _m	The number of measurements given by this sensor
	*/
	KalmanSensorCore(int _n, int _m);
	/** \brief Destructor */
	~KalmanSensorCore();
	/** \brief Accessor for n */
	int get_n() { return n; }
	/** \brief Accessor for m */
	int get_m() { return m; }
	/** \brief Method for updating the state estimate x
	* This is called from \e predict_update() of \e Kalman.
	* In \e KalmanSensorCore and in \e KalmanSensor this update is made linearly 
	* but \e KalmanSensorEkf will override this method to use unlinear estimation.
	*/
	virtual void update_x(CvMat *x_pred, CvMat *x);
};

/** \brief Core implementation for Kalman */
class ALVAR_EXPORT KalmanCore {
	friend class KalmanVisualize;
protected:
	int n; 
	//CvMat *x_pred;
	CvMat *F_trans;
	virtual void predict_x(unsigned long tick);
public:
	/** \brief The Kalman state vector (n*1) */
	CvMat *x;
	/** \brief The matrix (n*n) containing the transition model for the internal state.  */
	CvMat *F;
	/** \brief Copy constructor */
	KalmanCore(const KalmanCore &s);
	/** 
	* \brief 		Constructor
	* \param _n	The number of items in the Kalman  state vector
	*/
	KalmanCore(int _n);
	/** \brief Destructor */
	~KalmanCore();
	/** \brief Accessor for n */
	int get_n() { return n; }
	/** \brief Predict the Kalman state vector for the given time step .
	* 	x_pred = F * x
	*/
	virtual CvMat *predict();
	/** \brief Predict the Kalman state vector and update the state using the constant Kalman gain. 
	* 	x = x_pred + K* ( z - H*x_pred)
	*/
	CvMat *predict_update(KalmanSensorCore *sensor);

	/** \brief Predicted state, TODO: should be protected?! */
	CvMat *x_pred;

};

/** \brief Kalman sensor implementation */
class ALVAR_EXPORT KalmanSensor : public KalmanSensorCore {
protected:
	CvMat *R_tmp;
	CvMat *P_tmp;
public:
	/** \brief The covariance matrix for the observation noise */
	CvMat *R;
	/** \brief Copy constructor */
	KalmanSensor(const KalmanSensor &k);
	/** 
	* \brief 		Constructor
	* \param _n 		The number of items in the Kalman  state vector
	* \param _m	The number of measurements given by this sensor
	*/
	KalmanSensor(int n, int _m);
	/** \brief Destructor */
	~KalmanSensor();
	/** \brief Method for updating how the  Kalman state vector is mapped into this sensor's measurements vector.
	* This is called from \e predict_update() of \e Kalman.
	* Please override this method if you want this mapping to change on the run (e.g. based on time?).
	*/
	virtual void update_H(CvMat *x_pred) {}
	/** \brief Method for updating the  Kalman gain.
	* This is called from \e predict_update() of \e Kalman.
	*/
	virtual void update_K(CvMat *P_pred);
	/** \brief Method for updating the  error covariance matrix describing the accuracy of the state estimate.
	* This is called from \e predict_update() of \e Kalman.
	*/
	virtual void update_P(CvMat *P_pred, CvMat *P);
};

/** \brief Kalman implementation
*
* The Kalman filter provides an effective way of estimating a system/process recursively 
* (http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf). In this implementation
* we have separated the Kalman class (\e KalmanCore, \e Kalman or \e KalmanEkf)
* from the sensor class (\e KalmanSensorCore, \e KalmanSensor or \e KalmanSensorEkf).
* The selected Kalman class contains always the latest estimation of the process.
* The estimation can be updated using one or several sensors. This implementation allows
* SCAAT approach, where there may be several sensors (and several controls) for each 
* Kalman filter (See http://www.cs.unc.edu/~welch/scaat.html). 
* 
* Currently we have have three levels of implementations for both Kalman and Sensor (\e core, \e "normal" and \e EKF).
*
* The \e core implementations can be used for really fast bare-bones core implementations when 
* we have precalculated and constant \e K. In systems where \e F, \e H, \e Q and \e R are constants 
* the \e K will converge into a constant and it can be precalculated. Note, that the core 
* implementation need to assume constant frame rate if \e F depends on the timestep between frames. 
* Note also, that the core-classes cannot use \e EKF Jacobians because they change the \e H.
*
* The \e "normal" implementations are used when we have a linear \e F for \e Kalman, or linear \e H
* for \e KalmanSensor. The \e EKF implementations are used when we have non-linear function \e f()
* for \e KalmanEkf, or non-linear function \e h() for \e KalmanSensorEkf.
*
* Furthermore we have a class \e KalmanVisualize for visualizing the internal state of \e Kalman.
*
* Note, that now the \e KalmanControl is left out from this implementation. But it could be added 
* using similar conventions as the \e KalmanSensor.
*/
class ALVAR_EXPORT Kalman : public KalmanCore {
protected:
	int prev_tick; 
	void predict_P();
public:
	/** \brief The error covariance matrix describing the accuracy of the state estimate */
	CvMat *P;
	/** \brief The covariance matrix for the process noise */
	CvMat *Q;
	/** \brief The predicted error covariance matrix */
	CvMat *P_pred;
	/** 
	* \brief 		Constructor
	* \param n 		The number of items in the Kalman  state vector
	* \param _m	The number of measurements given by this sensor
	*/
	Kalman(int _n);
	/** \brief Destructor */
	~Kalman();
	/**
	* If your transition matrix F is based on time you need to override this method.
	*/
	virtual void update_F(unsigned long tick);
	/** \brief Predict the Kalman state vector for the given time step 
	* This calls \e updateF for updating the transition matrix based on the real time step
	*
	*  x_pred = F*x
	*  P_pred = F*P*trans(F) + Q
	*/
	CvMat *predict(unsigned long tick);
	/** \brief Predict the Kalman state vector for the given time step and update the state using the Kalman gain. 
	* - Calls first the predict to ensure that the prediction is based on same timestep as the update
	* - K = P_pred * trans(H) * inv(H*P_pred*trans(H) + R)
	* - x = x_pred + K* ( z - H*x_pred)
	* - P = (I - K*H) * P_pred
	*/
	CvMat *predict_update(KalmanSensor *sensor, unsigned long tick);
	/** \brief Helper method.  */
	double seconds_since_update(unsigned long tick);
};

/** \brief Extended Kalman Filter (EKF) sensor implementation.
*
* Please override the pure virtual \e h() with the desired unlinear function.
* By default the \e upate_H calculates the Jacobian numerically, if you want other approach override
* also the \e update_H()
*/
class ALVAR_EXPORT KalmanSensorEkf : public KalmanSensor {
protected:
	CvMat *delta;
	CvMat *x_plus;
	CvMat *x_minus;
	CvMat *z_tmp1;
	CvMat *z_tmp2;
	virtual void h(CvMat *x_pred, CvMat *_z_pred) = 0;
	virtual void update_H(CvMat *x_pred);
	virtual void update_x(CvMat *x_pred, CvMat *x);
public:
	KalmanSensorEkf(const KalmanSensorEkf &k);
	KalmanSensorEkf(int _n, int _m);
	~KalmanSensorEkf();
};

/** \brief Extended Kalman Filter (EKF) implementation.
*
* Please override the pure virtual \e f() with the desired unlinear function.
* By default the \e upate_F calculates the Jacobian numerically, if you want other approach override
* also the \e update_F()
*/
class ALVAR_EXPORT KalmanEkf : public Kalman {
protected:
	CvMat *delta;
	CvMat *x_plus;
	CvMat *x_minus;
	CvMat *x_tmp1;
	CvMat *x_tmp2;
	virtual void f(CvMat *_x, CvMat *_x_pred, double dt) = 0;
	virtual void update_F(unsigned long tick);
	virtual void predict_x(unsigned long tick);
public:
	KalmanEkf(int _n);
	~KalmanEkf();
};

/** \brief Class for visualizing Kalman filter

Usage:
\code
	KalmanVisualize kvis(&kalman, &sensor);
	...
	kvis.update_pre();
	kalman.predict_update(&sensor);
	kvis.update_post();
	kvis.show();
\endcode
*/
class ALVAR_EXPORT KalmanVisualize {
	int n;
	int m;
	KalmanCore *kalman;
	KalmanSensorCore *sensor;
	Kalman *kalman_ext;
	KalmanSensor *sensor_ext;
	/** \brief Image collecting visualization of the Kalman filter */
	IplImage *img;
	/** \brief Image to show */
	IplImage *img_legend;
	/** \brief Image to show */
	IplImage *img_show;
	/** \brief visualization scale before show */
	int img_scale;
	/** \brief Add matrix to the image */
	void img_matrix(CvMat *mat, int top, int left);
	/** \brief Init everything. Called from constructors. */
	void Init();
public:
	/** \brief Helper method for outputting matrices (for debug purposes) */
	static void out_matrix(CvMat *m, char *name);
	/** \brief Constructor for full Kalman implementation */
	KalmanVisualize(Kalman *_kalman, KalmanSensor *_sensor);
	/** \brief Constructor for core Kalman implementation (not all visualizations possible) */
	KalmanVisualize(KalmanCore *_kalman, KalmanSensorCore *_sensor);
	/** \brief Destructor */
	~KalmanVisualize();
	/** \brief Update the visualization image - call this before the Kalman's predict_update */
	void update_pre();
	/** \brief Update the visualization image - call this after the Kalman's predict_update */
	void update_post();
	/** \brief Show the genrated visualization image */
	void show();
};

} // namespace alvar 

#endif

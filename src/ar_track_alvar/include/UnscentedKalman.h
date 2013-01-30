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

#ifndef __UNSCENTED_KALMAN__
#define __UNSCENTED_KALMAN__

#include "Alvar.h"
#include "cxcore.h"

/**
 * \file UnscentedKalman.h
 *
 * \brief This file implements an unscented Kalman filter.
 */

namespace alvar {

  class UnscentedProcess;
  class UnscentedObservation;

  /**
   * \brief Implementation of unscented kalman filter (UKF) for filtering non-linear
   * processes.
   *
   * See http://www.cs.unc.edu/~welch/kalman/media/pdf/Julier1997_SPIE_KF.pdf
   * for more details about UKF.
   *
   * The UKF estimates a process state (represented by a vector) using observations
   * of the process. Observations are some derivate of the process state as usually 
   * the process state cannot be directly observed.
   *
   * \e UnscentedProcess models the process by predicting the next filter state
   * based on the current filter state.
   *
   * \e UnscentedObservation models the observation by predicting observation results
   * based on the current filter state.
   *
   * UnscentedKalman holds the estimated process state vector and its covariance
   * matrix. The new process state can be estimated using \e predict and \e update
   * methods.
   *
   * The current implementation does not separate process noise elements from
   * the process state vector. It is therefore the responsibility of the user
   * to include noise terms into process state and state covariance.
   *
   * \code
   *   class MyUnscentedProcess : public UnscentedProcess {
   *     void f(CvMat *state) { // compute new state }
   *     CvMat *getProcessNoise() { return _noise; }
   *   } myProcess;
   *
   *   class MyUnscentedObservation : public UnscentedObservation {
   *     void h(CvMat *z, cvMat *state) { // compute measurement vector z from state }
   *     CvMat *getObservation() { return _obs; }
   *     CvMat *getObservationNoise() { return _noise; }
   *   } myObservation;
   *
   *   int state_n = NUMBER_OF_ELEMENTS_IN_PROCESS_STATE_VECTOR;
   *   int obs_n = NUMBER_OF_ELEMENTS_IN_PROCESS_OBSERVATION_VECTOR;
   *   int state_k = NUMBER_OF_PROCESS_NOISE_ELEMENTS; //TODO: Not supported at the moment.
   *
   *   UnscentedKalman ukf(state_n, obs_n, state_k);
   *   initializeState(ukf.getState(), ukf.getStateCovariance());
   *   ukf.initialize();
   *
   *   while (1) {
   *     ukf.predict(&myProcess);
   *     // measure new observation.
   *     ukf.update(&myObservation);
   *     CvMat *state = ukf.getState();
   *     // unpack state information from the state vector and do something with it.
   *   }
   *
   * \endcode
   */
  class ALVAR_EXPORT UnscentedKalman {
  private:
    int state_n;
    int state_k;
    int obs_n;
    int sigma_n;
    bool sigmasUpdated;
	double lambda, lambda2;

    CvMat *state;
    CvMat *stateCovariance;
    CvMat *sqrtStateCovariance;
    CvMat *stateD;
    CvMat *stateU;
    CvMat *stateV;
    CvMat *stateTmp;
    CvMat *stateDiff;

    CvMat *predObs;
    CvMat *predObsCovariance;
    CvMat *invPredObsCovariance;
    CvMat *predObsDiff;

    CvMat *statePredObsCrossCorrelation;
    CvMat *kalmanGain;
    CvMat *kalmanTmp;

    CvMat **sigma_state;
    CvMat **sigma_predObs;

    // possess state mean and co-variance (as a list of sigma points).
    // generate sigma points from state mean vector and co-variance matrix.
    // compute state mean vector and co-variance matrix from sigma points.

    // predict: 
    //  - map sigma points thru process model f.

    // update:
    //  - map sigma points thru h.
    //  - from current sigma points and sigma observations:
    //   - compute state estimate x and co-variance P.
    //   - compute predicted observation z and innocation co-variance Z
    //   - compute cross correlation XZ
    //  - compute new state mean and co-variance.
    //  - generate new sigma points.
  public:

    /** \brief Initializes Unscented Kalman filter.
     *
     * Initializes Unscented Kalman filter. The state vector returned by \e getState
     * and state covariance matrix returned by \e getStateCovariance should be
     * initialized before using the filter.
     *
     * Separate state noise vector is currently unsupported. The user should include
     * noise terms in the state vector directly. Set the noise mean into state vector
     * and noise variance into state covariance matrix.
     *
     * \param state_n The number of elements in process state vector.
     * \param obs_n The number of elements in observation vector.
     * \param state_k The number of noise elements used in the process model.
     *                TODO: This is currently unsupported.
	 * \param alpha Spread of sigma points.
	 * \param beta Prior knowlegde about the distribution (2 for Gaussian).
     */
    UnscentedKalman(int state_n, int obs_n, int state_k = 0, double alpha = 0.001, double beta = 2.0);
    ~UnscentedKalman();

    /** \brief Returns the process state vector.
     * 
     * The returned state vector contains the current state of the process.
     * The returned vector may be modified if the current process state is
     * known, for example in initialization phase. If the vector is modified,
     * \e initialize method must be called before calling either predict or
     * update methods.
     *
     * \return A vector of state_n elements.
     */
    CvMat *getState() { return state; }

    /** \brief Returns the process state covariance matrix.
     *
     * The returned matrix contains the current state covariance. The matrix
     * may be modified if the covariance is known, for example in initialization
     * phase. If the matrix is modified, \e initialize method must be called
     * before calling either predict of update methods.
     *
     * \return state_n by state_n covariance matrix.
     */
    CvMat *getStateCovariance() { return stateCovariance; }

    /** \brief (Re-)initialize UKF internal state.
     *
     * Must be called before predict/update when ever state or state co-variance
     * are changed.
     */
    void initialize();

    /** \brief Updated the state by predicting.
     *
     * Updates the process state by predicting new state from the current state.
     * Normally each predict call is followed with a call to update method.
     *
     * \param process_model The model implementation that is used to predict the
     *        next state.
     */
    void predict(UnscentedProcess *process_model);

    /** \brief Updates the state by an observation.
     *
     * Updates the process state by a measurement that indirectly observed the
     * correct process state. The observation implementation needs to hold the
     * current measurement data and implement a transformation from process state
     * into measurement (the \e UnscentedObservation::h method).
     *
     * \param observation The observation implementation the is used to update
     *        the current state.
     */
    void update(UnscentedObservation *observation);
  };

  /**
   * \brief Process model for an unscented kalman filter.
   *
   * Implementing class needs to allocate a noise matrix of correct size.
   */
  class ALVAR_EXPORT UnscentedProcess {
  public:
    /** \brief process model: state+1 = f(state)
     *
     * Model the process by computing an estimate how the process changes
     * when one timestep is taken.
     *
     * \param state state_n size vector; The current state in input and the next
     *              state estimate in output.
     */
    virtual void f(CvMat *state) = 0;

    /** \brief Returns the process noise covariance.
     *
     * The returned matrix will be added to the current state covariance matrix,
     * increasing the uncertainty of the current state. The matrix should reflect
     * all unknown factors of the process which are not taken into account by the
     * state estimation method \e f.
     *
     * \return state_n by state_n size matrix; or NULL for no additional noise.
     */
    virtual CvMat *getProcessNoise() = 0;
  };

  /**
   * \brief Observation model for an unscented kalman filter.
   *
   * The implementation needs to  allocate correct size measurement vector and
   * noise matrix and to implement a transformation from process state into a
   * measurement.
   */
  class ALVAR_EXPORT UnscentedObservation {
  public:
    /** \brief observation model: z = h(state)
     *
     * Computes an estimated measurement vector from the current state estimate.
     *
     * \param z obs_n size vector; The estimated measurement.
     * \param state state_n size vector; The current state.
     */
    virtual void h(CvMat *z, CvMat *state) = 0;

    /** \brief Returns the current measurement vector.
     *
     * The returned vector should contain the latest measurement values. 
     * In the UKF update phase the process state will be modified in such a
     * way to make the difference between estimated measurement (from method \e h)
     * and the returned real measurement smaller.
     * 
     * \return obs_n size vector containing the current measured values.
     */
    virtual CvMat *getObservation() = 0;

    /** \brief Returns the observation noise covariance matrix.
     *
     * The returned matrix will be added to the current observation covariance
     * matrix increasing the uncertainty of measurements. The matrix should
     * reflect the amount of noise in the measurement vector returned by \e
     * getObservation method.
     *
     * \return obs_n by obs_b matrix containing observation noise covariance; or
     *         NULL for no additional noise.
     */
    virtual CvMat *getObservationNoise() = 0;
  };

} // namespace alvar

#endif // __UNSCENTED_KALMAN__

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

#ifndef __Ransac_h__
#define __Ransac_h__

#include "Alvar.h"
#include <stdlib.h>

/**
 * \file Ransac.h
 *
 * \brief This file implements a generic RANSAC algorithm.
 */

namespace alvar {

/**
 * \brief Internal implementation of RANSAC. Please use Ransac or IndexRansac.
 */
class ALVAR_EXPORT RansacImpl {

 protected:
  void** samples;
  void* hypothesis;
  int min_params;
  int max_params;
  int sizeof_param;
  int sizeof_model;

  RansacImpl(int min_params, int max_params, 
             int sizeof_param, int sizeof_model);
  virtual ~RansacImpl();

  int _estimate(void* params, int param_c,
		int support_limit, int max_rounds,
		void* model);

  int _refine(void* params, int param_c,
	      int support_limit, int max_rounds,
	      void* model, char *inlier_mask = NULL);

  virtual void _doEstimate(void** params, int param_c, void* model) {};
  virtual bool _doSupports(void* param, void* model) { return false; };

  /** IndexRansac version */
  int *indices;

  RansacImpl(int min_params, int max_params, 
             int sizeof_model);

  int _estimate(int param_c,
		int support_limit, int max_rounds,
		void* model);

  int _refine(int param_c,
	      int support_limit, int max_rounds,
	      void* model, char *inlier_mask = NULL);

  virtual void _doEstimate(int* params, int param_c, void* model) {};
  virtual bool _doSupports(int param, void* model) { return false; };

 public:

  /** \brief How many rounds are needed for the Ransac to work.
   *
   * Computes the required amount of rounds from the estimated
   * inlier percentage and required propability of successfully
   * finding the inlier set.
   *
   * \param success_propability Required success propability,
   * (range 0..1)
   * \param inlier_percentage Estimated amount of inliers in the
   * parameter set, range(0..1).
   * \return The required number of rounds that can be used as
   * max_rounds when calling the estimate method.
   */
  int estimateRequiredRounds(float success_propability,
			     float inlier_percentage);

};

  /**
   * \brief Implementation of a general RANdom SAmple Consensus algorithm.
   *
   * This implementation can be used to estimate model from a set of input
   * data. The user needs to provide support methods to compute the best
   * model given a set of input data and to classify input data into inliers
   * and outliers.
   *
   * For more information see "Martin A Fischler and Robrt C. Bolles:
   * Random Sample Consensus: a paradigm for model fitting with applications
   * to image analysis and automated cartography. Comm of the ACM 24: 381-395"
   * (http://portal.acm.org/citation.cfm?doid=358669.358692).
   * 
   * MODEL is the estimated model, for example endpoints of a line for
   * line fitting.
   *
   * PARAMETER is the input for model estimation, for example 2D point
   * for line fitting.
   *
   * MODEL must support an assigment operator.
   *
   * The user needs to extend this class and provide two methods:
   * \code
   *   void doEstimate(PARAMETER** params, int param_c, MODEL* model);
   *   bool doSupports(PARAMETER* param, MODEL* model);
   * \endcode
   *
   * Example: Fitting points to a line:
   *
   * \code
   * typedef struct Point { double x, double y; };
   * typedef struct Line { Point p1, p2; };
   *
   * class MyLineFittingRansac : public Ransac<Line, Point> {
   *   // Line fitting needs at least 2 parameters and here we want
   *   // to support at most 16 parameters.
   *   MyLineFittingRansac() : Ransac(2, 16) {}
   *
   *   void doEstimate(Point **points, int points_c, Line *line) {
   *     if (points_c == 2) { return Line(*points[0], *points[1]); }
   *     else { // compute best line fitting up to 16 points. }
   *   }
   *
   *   bool doSupports(Point *point, Line *line) {
   *     double distance = // compute point distance to line.
   *     return distance < POINT_DISTANCE_LIMIT;
   *   }
   * };
   *
   * Point input[N_POINTS] = { .. };
   * Line line;
   * MyLineFittingRansac ransac;
   *
   * // assuming 60% of inliers, run RANSAC until the best model is found with 99% propability.
   * int max_rounds = ransac.estimateRequiredRounds(0.99, 0.6);
   * int number_of_inliers = ransac.estimate(input, N_POINTS, N_POINTS, max_rounds, &line);
   *
   * // lets make the estimate even better.
   * if (number_of_inliers > 0 && number_of_inliers < N_POINTS)
   *   number_of_inliers = ransac.refine(input, N_POINTS, N_POINTS, 10, &line);
   *
   * // you should keep track of the real percentage of inliers to determine
   * // the  required number of RANSAC rounds.
   * double inlier_percentage = (double)number_of_inliers / (double)N_POINTS;
   *
   * \endcode
   */
  template <typename MODEL, typename PARAMETER>
    class Ransac : public RansacImpl {

    protected:
    /** \brief Creates a model estimate from a set of parameters.
     *
     * The user must implement this method to compute model parameters
     * from the input data.
     *
     * \param params An array of pointers to sampled parameters (input data).
     * \param param_c The number of parameter pointers in the params array.
     * \param model Pointer to the model where to store the estimate.
     */
    virtual void doEstimate(PARAMETER** params, int param_c, MODEL* model) = 0;

    /** \brief Computes how well a parameters supports a model.
     *
     * This method is used by the RANSAC algorithm to count how many
     * parameters support the estimated model (inliers). Althought this
     * is case specific, usually parameter supports the model when the distance
     * from model prediction is not too far away from the parameter.
     *
     * \param param Pointer to the parameter to check.
     * \param model Pointer to the model to check the parameter against.
     * \return True when the parameter supports the model.
     */
    virtual bool doSupports(PARAMETER* param, MODEL* model) = 0;

    /**
     * Wrapper for templated parameters.
     */
    void _doEstimate(void** params, int param_c, void* model) {
      doEstimate((PARAMETER**)params, param_c, (MODEL*)model);
    }

    /**
     * Wrapper for templated parameters.
     */
    bool _doSupports(void* param, void* model) {
      return doSupports((PARAMETER*)param, (MODEL*)model);
    }

    public:
    /** \brief Initialize the algorithm.
     *
     * Uses at least min_params and at most max_params number of input data
     * elements for model estimation.
     *
     * Must be: max_params >= min_params
     *
     * \param min_params is the minimum number of parameters needed to create
     * a model.
     * \param max_params is the maximum number of parameters to using in refining
     * the model.
     */
    Ransac(int min_params, int max_params) 
      : RansacImpl(min_params, max_params, sizeof(PARAMETER), sizeof(MODEL)) {}

    virtual ~Ransac() {}

    /** \brief Estimates a model from input data parameters.
     *
     * Randomly samples min_params number of input data elements from params array
     * and chooses the model that has the largest set of supporting parameters
     * (inliers) in the params array.
     *
     * Note that this method always uses min_params number of parameters, that is,
     * \e doEstimate method can be implemented to support only the minimum number
     * of parameters unless \e refine method is used.
     *
     * \param params Parameters that the model is estimated from (input data).
     * \param param_c Number of elements in the params array.
     * \param support_limit The search is stopped if a model receives
     * more support that this limit.
     * \param max_rounds How many different samples are tried before
     * stopping the search.
     * \param model The estimated model is stored here.
     * \return the number of parameters supporting the model, or 0
     * if a suitable model could not be build at all.
     */
    int estimate(PARAMETER* params, int param_c,
		 int support_limit, int max_rounds,
		 MODEL* model) {
      return _estimate(params, param_c, support_limit, max_rounds, model);
    }

    /** \brief Iteratively makes the estimated model better.
     *
     * Starting with the estimated model, computes the model from all
     * inlier parameters and interates until no new parameters support
     * the model.
     *
     * Note that this method uses up to max_params number of parameters,
     * that is, \e doEstimate method must be implemented in such a way
     * that it can estimate a model from a variable number of parameters.
     *
     * \param params Parameters that the model is estimated from.
     * \param param_c Number of parameters.
     * \param support_limit The search is stopped is a model receives
     * more support that this limit.
     * \param max_rounds How many iterations of the refinement are run.
     * \param model The estimated model that is refined.
     * \param inlier_mask Byte array where 1 is stored for inliers and 0 for outliers.
     * \return the number of parameters supporting the model.
     */
    int refine(PARAMETER* params, int param_c,
	       int support_limit, int max_rounds,
	       MODEL* model, char *inlier_mask = NULL) {
      return _refine(params, param_c, support_limit, max_rounds, model, inlier_mask);
    }

  };


  /**
   * \brief Implementation of a general RANdom SAmple Consensus algorithm
   * with implicit parameters.
   * 
   * These parameters are accessed by indises. The benefit of this is that
   * we avoid copying input data from an array into another. See \e Ransac
   * class for more details.
   *
   * Extending class must provide two methods:
   * \code
   *   void doEstimate(int* params, int param_c, MODEL* model);
   *   bool doSupports(int param, MODEL* model);
   * \endcode
   *
   * Example fitting points to a line (compare this with the example
   * in the \e Ransac class):
   *
   * \code
   * typedef struct Point { double x, double y; };
   * typedef struct Line { Point p1, p2; };
   *
   * class MyLineFittingRansac : public IndexRansac<Line, Point> {
   *   Point *points;
   *
   *   // Line fitting needs at least 2 parameters and here we want
   *   // to support at most 16 parameters.
   *   MyLineFittingRansac(Points *input) : Ransac(2, 16), points(input) {}
   *
   *   void doEstimate(int *indises, int points_c, Line *line) {
   *     if (points_c == 2) { return Line(points[indises[0]], *points[indises[1]]); }
   *     else { // compute best line fitting up to 16 points. }
   *   }
   *
   *   bool doSupports(int index, Line *line) {
   *     Point *point = &points[index];
   *     double distance = // compute point distance to line.
   *     return distance < POINT_DISTANCE_LIMIT;
   *   }
   * };
   *
   * Point input[N_POINTS] = { .. };
   * Line line;
   * MyLineFittingRansac ransac(input);
   *
   * // assuming 60% of inliers, run RANSAC until the best model is found with 99% propability.
   * int max_rounds = ransac.estimateRequiredRounds(0.99, 0.6);
   * int number_of_inliers = ransac.estimate(input, N_POINTS, N_POINTS, max_rounds, &line);
   *
   * // lets make the estimate even better.
   * if (number_of_inliers > 0 && number_of_inliers < N_POINTS)
   *   number_of_inliers = ransac.refine(input, N_POINTS, N_POINTS, 10, &line);
   *
   * // you should keep track of the real percentage of inliers to determine
   * // the  required number of RANSAC rounds.
   * double inlier_percentage = (double)number_of_inliers / (double)N_POINTS;
   *
   * \endcode
   */
  template <typename MODEL>
    class IndexRansac : public RansacImpl {

  protected:
    /** \brief Creates a model estimate from a set of parameters.
     *
     * The user must implement this method to compute model parameters
     * from the input data.
     *
     * \param params An array of indises of sampled parameters.
     * \param param_c The number of parameter indises in the params array.
     * \param model Pointer to the model where to store the estimate.
     */
    virtual void doEstimate(int* params, int param_c, MODEL* model) = 0;

    /** \brief Computes how well a parameters supports a model.
     *
     * This method is used by the RANSAC algorithm to count how many
     * parameters support the estimated model (inliers). Althought this
     * is case specific, usually parameter supports the model when the distance
     * from model prediction is not too far away from the parameter.
     *
     * \param param Index of the parameter to check.
     * \param model Pointer to the model to check the parameter against.
     * \return True when the parameter supports the model.
     */
    virtual bool doSupports(int param, MODEL* model) = 0;

    /**
     * Wrapper for templated parameters.
     */
    void _doEstimate(int* params, int param_c, void* model) {
      doEstimate(params, param_c, (MODEL*)model);
    }

    /**
     * Wrapper for templated parameters.
     */
    bool _doSupports(int param, void* model) {
      return doSupports(param, (MODEL*)model);
    }

  public:
    /** \brief Initialize the algorithm.
     *
     * Uses at least min_params and at most max_params number of input data
     * elements for model estimation.
     *
     * Must be: max_params >= min_params
     *
     * \param min_params is the minimum number of parameters needed to create
     * a model.
     * \param max_params is the maximum number of parameters to using in refining
     * the model.
     */
    IndexRansac(int min_params, int max_params) 
      : RansacImpl(min_params, max_params, sizeof(MODEL)) {}

    virtual ~IndexRansac() {}

    /** \brief Estimates a model from input data parameters.
     *
     * Randomly samples min_params number of input data elements from params array
     * and chooses the model that has the largest set of supporting parameters
     * (inliers) in the params array.
     *
     * Note that this method always uses min_params number of parameters, that is,
     * \e doEstimate method can be implemented to support only the minimum number
     * of parameters unless \e refine method is used.
     *
     * \param param_c Number of parameters available in estimation.
     * \param support_limit The search is stopped if a model receives
     * more support that this limit.
     * \param max_rounds How many different samples are tried before
     * stopping the search.
     * \param model The estimated model is stored here.
     * \return the number of parameters supporting the model, or 0
     * if a suitable model could not be build at all.
     */
    int estimate(int param_c,
		 int support_limit, int max_rounds,
		 MODEL* model) {
      return _estimate(param_c, support_limit, max_rounds, model);
    }

    /** \brief Iteratively makes the estimated model better.
     *
     * Starting with the estimated model, computes the model from all
     * inlier parameters and interates until no new parameters support
     * the model.
     *
     * Note that this method uses up to max_params number of parameters,
     * that is, \e doEstimate method must be implemented in such a way
     * that it can estimate a model from a variable number of parameters.
     *
     * \param param_c Number of parameters available for estimation.
     * \param support_limit The search is stopped if a model receives
     * more support that this limit.
     * \param max_rounds How many iterations of the refinement are run.
     * \param model The estimated model that is refined.
     * \param inlier_mask Byte array where 1 is stored for inliers and 0 for outliers.
     * \return the number of parameters supporting the model.
     */
    int refine(int param_c,
	       int support_limit, int max_rounds,
	       MODEL* model, char *inlier_mask = NULL) {
      return _refine(param_c, support_limit, max_rounds, model, inlier_mask);
    }

  };

} // namespace alvar

#endif //__Ransac_h__

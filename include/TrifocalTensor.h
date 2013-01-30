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

#ifndef __TRIFOCAL_TENSOR__
#define __TRIFOCAL_TENSOR__

#include "Alvar.h"
#include <cxcore.h>
#include "Pose.h"

/**
 * \file TrifocalTensor.h
 *
 * \brief This file implements a trifocal tensor.
 */

namespace alvar {

  /**
   * \brief Trifocal tensor works for three images like a fundamental matrix
   * works for two images.
   * 
   * Given three camera poses P0, P1, P2 and a 3D point X, we can calculate a
   * trifocal tensor T(P0, P1, P2). The tensor relates projections of X in P0,
   * P1 and P2 in such a way that when any two projections are known the third
   * projection can be calculated.
   *
   * This implementation of trifocal tensor assumes that the camera poses
   * P0, P1 and P2 are known. When projections of X in P0 and P1 are known
   * the projection in P2 can be computed using the tensor.
   *
   * The current implementation cannot be used to directly compute the tensor
   * from point correspondencies alone. The implementation can be used for 
   * example for optimizing three camera poses when point correspondences are
   * known in the three images by minimizing the trifocal 'projection error'
   * computed by \e projectError -method.
   *
   * \code
   *   Pose P1, P2; // P0 is identity pose.
   *   CvPoint2D64f proj0, proj1, proj2; // A 3D point projected with poses P0, P1 and P2.
   *
   *   TrifocalTensor T(P1, P2);
   *   CvPoint2D64f test2;
   *   T.project(proj0, proj1, test2);
   *   assert(proj2.x == test2.x);
   *   assert(proj2.y == test2.y);
   *   assert(proj2.z == test2.z);
   * \endcode
   */
  class ALVAR_EXPORT TrifocalTensor {
  private:
    double T[3][3][3];
    double projectAxis(const CvPoint2D64f &p0, const CvPoint2D64f &p1, int l);

  public:
    TrifocalTensor();

    /** \brief Constructs a tensor from identity pose and two other known poses.
     * See \e computeTensor.
     *
     * \param P1 The second pose relative to the first pose.
     * \param P2 The third pose relative to the first pose.
     */
    TrifocalTensor(const Pose &P1, const Pose &P2);

    /** \brief Constructs a tensor from three known poses.
     * See \e computeTensor.
     *
     * \param P0 The first camera pose.
     * \param P1 The second pose relative to the first pose.
     * \param P2 The third pose relative to the first pose.
     */
    TrifocalTensor(const Pose &P0, const Pose &P1, const Pose &P2);

    ~TrifocalTensor();
  
    /** \brief Initializes the tensor from identity pose and two other known 
     * poses.
     *
     * The first pose is identity and the two other poses are relative 
     * translations/rotation between the first and the second pose and
     * between the first and the third pose.
     *
     * \param P1 The second pose relative to the first pose.
     * \param P2 The third pose relative to the first pose.
     */
    void computeTensor(const Pose &P1, const Pose &P2);
  
    /** \brief Initializes the tensor from three known poses.
     *
     * \param P0 The first camera pose.
     * \param P1 The second pose relative to the first pose.
     * \param P2 The third pose relative to the first pose.
     */
    void computeTensor(const Pose &P0, const Pose &P1, const Pose &P2);
  
    /** \brief Computes the projection of a point in the third pose.
     *
     * When we have three images, each a projection of a scene from the three
     * different camera poses (identity and the two poses that were used to
     * initialize the tensor) and a 2D point correspondence between the first
     * and the second images, a position in the third image is computed.
     *
     * \param p0 2D position in a image projected from the first pose.
     * \param p1 2D position in a image projected from the second pose.
     * \param p2 Computed 2D position in a image projected form the third pose.
     */
    void project(const CvPoint2D64f &p0, const CvPoint2D64f &p1, CvPoint2D64f &p2);
  
    /** \brief Computes how much three points differ from the tensor.
     *
     * When we have three images, each a projection of a scene from the three
     * different camera poses (identity and the two poses that were used to
     * initialize the tensor) and a 2D point correspondence between the first
     * the second and the third images, we compute how well the three projections
     * match with the trifocal tensor.
     *
     * When the third point lies exactly where the tensor projects the first
     * two points, the returned error is zero.
     *
     * \param p0 2D position in a image projected from the first pose.
     * \param p1 2D position in a image projected from the second pose.
     * \param p2 2D position in a image projected form the third pose.
     * \return Squared projection error.
     */
    double projectError(const CvPoint2D64f &p0, const CvPoint2D64f &p1, const CvPoint2D64f &p2);
  };

} // namespace alvar

#endif // __TRIFOCAL_TENSOR__

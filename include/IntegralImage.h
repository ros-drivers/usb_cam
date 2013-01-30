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

#ifndef INTEGRAL_IMAGE_H
#define INTEGRAL_IMAGE_H

/**
 * \file IntegralImage.h
 *
 * \brief This file implements integral image and integral gradient
 * computations.
 */

#include "Alvar.h"
#include <cxcore.h>
#include <cv.h>

namespace alvar {

/** \brief Class for calculating "evenly spaced" integer indices for data sequence
 *
 * If we have a data sequence we want to step through in certain amount of steps,
 * \e IntIndex can be used for iterating through the steps using fast integer implementation.
 * This class is related to stride iterators.
 *
 * \section Usage
 * \code
 * int data[x_res];
 * for (IntIndex x(x_res, sub_res); x.get() != x.end(); x.next()) {
 *     std::cout<<"data["<<x.get()<<"] is "<<data[x.get()]<<std::endl;
 *     ...
 * }
 * \endcode
 */
class ALVAR_EXPORT IntIndex {
protected:
    int index;
    int step;
    int step_remainder;
    int estep;
    int next_step;
    int res;
    int steps;
    void update_next_step();
public:
    /** \brief Create \e IntIndex for indexing \e _res elements in predefined amount of \e _steps . 
     *  \param _res The number of elements in the data sequence we want to index.
     *  \param _steps The number of steps to use to cover the \e _res elements (\e _steps < \e _res)
     */
    IntIndex(int _res, int _steps);
    /** \brief Set the integer index to the "grid" value nearest to \e v . 
     */
    int operator=(int v);
    /** \brief Take the next integer index step. */
    int next();
    /** \brief Get the index value */
    int get() const;
    /** \brief How much the index will be increased with the next \e next() */
    int get_next_step() const;
    /** \brief For testing have we reached the end. */
    int end() const;
};


/** \brief \e IntegralImage is used for calculating rectangular image sums and averages rapidly
 *
 * The integral images are based on making intermediate representation of the image. Using this
 * approach the sum/average of rectangular area can be calculated using only four references for
 * the integral image. The integral images are commonly used with HAAR-like features.
 *
 * The \e IntegralImage should be used when we need to a lot of sum/average calculations for
 * the same image.
 * 
 * \section References 
 * - Konstantinos G. Derpanis (2007). Integral image-based representations. Department of Computer 
 *   Science and Engineering, York University.
 * - Viola, P. & Jones, M. (2001). Rapid object detection using a boosted cascade of simple
 *   features. In IEEE Computer Vision and Pattern Recognition (pp. I:511-518).
 */
class ALVAR_EXPORT IntegralImage {
	IplImage *sum;
public:
	IntegralImage();
	~IntegralImage();
	/** \brief Update integral image for the given image.
	 *  \param gray The original grayscale image we want analyze
	 */
	void Update(IplImage *gray);
	/** \brief Calculate the sum for the given rectangular area in the image.
	 *  \param rect The rectancle
	 *  \param count If this parameter is not 0 it is filled with number of pixels in the rectangle.
	 */
    double GetSum(CvRect &rect, int *count=0);
	/** \brief Calculate the average for the given rectangular area in the image. */
	double GetAve(CvRect &rect);
	/** \brief Get a sub-image using integral image representation.
	 *  \param rect The rectangle we want to get the sub-image from
	 *  \param sub  The image where the sub-image is generated. Note, the desired resolution is defined by \e sub.
	 *
	 *  Get an image \e sub with a predefined resolution from the given 
	 *  rectangular area \e rect . In practice the \e sub is filled by
	 *  getting the average with \e GetAve() for every pixel area.
	 */
    void GetSubimage(const CvRect &rect, IplImage *sub);
};

/** \brief \e IntegralGradient is used for calculating rectangular image gradients rapidly
 *
 * We calculate \e IntegralImage:s based on point normals for 4-pixel intersections
 * (see Donahue1992). Using the integral images it is possible to make fast gradient
 * calculations to any image rectangle. This approach is useful when we need to calculate many
 * gradient rectangles for the same image.
 *
 * (See \e SampleIntegralImage.cpp)
 */
class ALVAR_EXPORT IntegralGradient {
protected:
	IplImage *normalx;
	IplImage *normaly;
	IntegralImage integx;
	IntegralImage integy;
	// Calculate point normals for 4-pixel intersection
	// as described in Donahue1992
	void CalculatePointNormals(IplImage *gray);
public:
	IntegralGradient();
	~IntegralGradient();
	/** \brief Update intermediate images for calculating the gradients to the given image.
	 *  \param gray The original grayscale image we want analyze
	 */
	void Update(IplImage *gray);
	/** \brief Calculate the gradient for the given rectangular area in the image.
	 *  \param dirx Method fills in the x-component of the gradient here
	 *  \param diry Method fills in the y-component of the gradient here
	 *  \param count If this parameter is not 0 it is filled with number of pixels in the rectangle.
	 */
	void GetGradient(CvRect &rect, double *dirx, double *diry, int *count=0);
	/** \brief Calculate the average gradient for the given rectangular area in the image.
	 *  \param dirx Method fills in the x-component of the gradient here
	 *  \param diry Method fills in the y-component of the gradient here
	 */
	void GetAveGradient(CvRect &rect, double *dirx, double *diry);
};

} // namespace alvar

#endif

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

#ifndef CONNECTEDCOMPONENTS_H
#define CONNECTEDCOMPONENTS_H

/**
 * \file ConnectedComponents.h
 *
 * \brief This file implements connected component labeling.
 */

#include "Alvar.h"
#include "Util.h"
#include "Line.h"
#include "Camera.h"

namespace alvar {

/**
 * \brief Connected components labeling methods.
*/
enum ALVAR_EXPORT LabelingMethod
{
	CVSEQ
};

/**
 * \brief Base class for labeling connected components from binary image.
*/
class ALVAR_EXPORT Labeling
{

protected :

	Camera	 *cam;
	int thresh_param1, thresh_param2;

public :

	/**
	 * \brief Pointer to grayscale image that is thresholded for labeling.
	*/
	IplImage *gray;
	/**
	 * \brief Pointer to binary image that is then labeled.
	*/
	IplImage *bw;

	/**
	 * \brief Vector of 4-length vectors where the corners of detected blobs are stored.
	*/
	std::vector<std::vector<PointDouble> > blob_corners;

	/**
	 * \brief Two alternatives for thresholding the gray image. ADAPT (adaptive threshold) is only supported currently.
	*/
	enum ThresholdMethod 
	{
		THRESH,
		ADAPT
	};

	/** Constructor */
	Labeling();

	/** Destructor*/
	~Labeling();

	/**
	 * \brief Sets the camera object that is used to correct lens distortions.
	*/
	void SetCamera(Camera* camera) {cam = camera;}

	/**
	 * \brief Labels image and filters blobs to obtain square-shaped objects from the scene.
	*/
	virtual void LabelSquares(IplImage* image, bool visualize=false) = 0;

	bool CheckBorder(CvSeq* contour, int width, int height);

	void SetThreshParams(int param1, int param2)
	{
		thresh_param1 = param1;
		thresh_param2 = param2;
	}
};

/**
 * \brief Labeling class that uses OpenCV routines to find connected components.
*/
class ALVAR_EXPORT LabelingCvSeq : public Labeling
{

protected :

	int _n_blobs;
	int _min_edge;
	int _min_area;
	bool detect_pose_grayscale;

	CvMemStorage* storage;

public:

	LabelingCvSeq();
	~LabelingCvSeq();

	void SetOptions(bool _detect_pose_grayscale=false);

	void LabelSquares(IplImage* image, bool visualize=false);

	// TODO: Releases memory inside, cannot return CvSeq*
	CvSeq* LabelImage(IplImage* image, int min_size, bool approx=false);
};

} // namespace alvar

#endif

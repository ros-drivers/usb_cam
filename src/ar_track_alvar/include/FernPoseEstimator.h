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

#ifndef FERNPOSEESTIMATOR_H
#define FERNPOSEESTIMATOR_H

/**
 * \file FernPoseEstimator.h
 *
 * \brief This file implements a pose estimator for the Fern-based image
 * detector.
 */

#include <map>
#include <vector>

#include "cv.h"

#include "Pose.h"
#include "Camera.h"
#include "EC.h"

namespace alvar
{

/**
 * \brief Pose estimation class for FernImageDetector.
 */
class ALVAR_EXPORT FernPoseEstimator
{
	
public:
	FernPoseEstimator();
    ~FernPoseEstimator();

    Pose pose() const;
    Camera camera() const;

    bool setCalibration(const std::string &filename, int width, int height);
    void setResolution(int width, int height);
	
    typedef std::vector<CvPoint2D64f> ImagePointVector;
    typedef std::vector<CvPoint3D64f> ModelPointVector;
    typedef std::map<int, alvar::ExternalContainer> ExternalContainerMap;
    void calculateFromPointCorrespondences(ModelPointVector &mpts, ImagePointVector &ipts);
    void updateFromTrackedPoints(ExternalContainerMap &container);
    void extractPlaneCoordinates(ExternalContainerMap &container);

private:
	Pose mPose;
	Camera mCamera;
	CameraEC mCameraEC;
};

} // namespace alvar

#endif

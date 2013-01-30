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

#include "FernPoseEstimator.h"

namespace alvar {

FernPoseEstimator::FernPoseEstimator()
    : mPose()
    , mCamera()
    , mCameraEC()
{
}

FernPoseEstimator::~FernPoseEstimator()
{
}

Pose FernPoseEstimator::pose() const
{
	return mPose;
}

Camera FernPoseEstimator::camera() const
{
	return mCamera;
}

bool FernPoseEstimator::setCalibration(const std::string &filename, int width, int height)
{
    bool r1 = mCamera.SetCalib(filename.c_str(), width, height);
	bool r2 = mCameraEC.SetCalib(filename.c_str(), width, height);
    return r1 && r2;
}

void FernPoseEstimator::setResolution(int width, int height)
{
    mCamera.SetRes(width, height);
	mCameraEC.SetRes(width, height);
}

void FernPoseEstimator::calculateFromPointCorrespondences(FernPoseEstimator::ModelPointVector &mpts, FernPoseEstimator::ImagePointVector &ipts)
{
	mCamera.CalcExteriorOrientation(mpts, ipts, &mPose); // TODO replace camera->cameraec
}

void FernPoseEstimator::updateFromTrackedPoints(FernPoseEstimator::ExternalContainerMap &container)
{
	mCameraEC.UpdatePose(container, &mPose);
}

void FernPoseEstimator::extractPlaneCoordinates(FernPoseEstimator::ExternalContainerMap &container)
{
	ExternalContainerMap::iterator iter = container.begin();
	ExternalContainerMap::iterator iter_end = container.end();
	for(; iter != iter_end; ++iter) {
		alvar::ExternalContainer &f = iter->second;
		mCameraEC.Get3dOnPlane(&mPose, f.p2d, f.p3d);
		f.has_p3d = true;
	}
}

} // namespace alvar

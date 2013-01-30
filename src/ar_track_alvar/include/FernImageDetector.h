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

#ifndef FERNIMAGEDETECTOR_H
#define FERNIMAGEDETECTOR_H

/**
 * \file FernImageDetector.h
 *
 * \brief This file implements a Fern-based image detector.
 */

#include <map>
#include <vector>
#include <iostream>
#include <fstream>

#include "cv.h"
#include "highgui.h"
#include "cvaux.h"

#include "Camera.h"
#include "EC.h"

using namespace cv;

namespace alvar{

/**
 * \brief FernClassifier subclass that implements binary reading and writting.
 */
class FernClassifierWrapper : public FernClassifier
{
public:
    FernClassifierWrapper();
    FernClassifierWrapper(const FileNode &fileNode);
    FernClassifierWrapper(const vector<vector<Point2f> > &points,
                          const vector<Mat> &referenceImages,
                          const vector<vector<int> > &labels = vector<vector<int> >(),
                          int _nclasses = 0,
                          int _patchSize = PATCH_SIZE,
                          int _signatureSize = DEFAULT_SIGNATURE_SIZE,
                          int _nstructs = DEFAULT_STRUCTS,
                          int _structSize = DEFAULT_STRUCT_SIZE,
                          int _nviews = DEFAULT_VIEWS,
                          int _compressionMethod = COMPRESSION_NONE,
                          const PatchGenerator &patchGenerator = PatchGenerator());
    virtual ~FernClassifierWrapper();

    virtual void readBinary(std::fstream &stream);
    virtual void writeBinary(std::fstream &stream) const;
};

/**
 * \brief Image detector based on a Fern classifier.
 */
class ALVAR_EXPORT FernImageDetector
{
public:	
    FernImageDetector(const bool visualize = false);
    ~FernImageDetector();
	
    void imagePoints(vector<CvPoint2D64f> &points);
    void modelPoints(vector<CvPoint3D64f> &points, bool normalize = true);

    cv::Size size();
    cv::Mat homography();
    double inlierRatio();

    void train(const std::string &filename);
    void train(Mat &image);
    void findFeatures(Mat &image, bool planeAssumption = true);
	
    bool read(const std::string &filename, const bool binary = true);
    bool write(const std::string &filename, const bool binary = true);

private:
    PatchGenerator mPatchGenerator;
    LDetector mLDetector;
    std::vector<FernClassifierWrapper> mClassifier;

    vector<KeyPoint> mKeyPoints;
    vector<cv::Point2f> mImagePoints;
    vector<cv::Point2f> mModelPoints;

    bool mVisualize;
    std::vector<Mat> mObjects;
    cv::Size mSize;
    cv::Mat mCorrespondences;
    cv::Mat mHomography;
    double mInlierRatio;
};

} // namespace alvar

#endif

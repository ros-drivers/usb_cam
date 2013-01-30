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

#include "FernImageDetector.h"

namespace alvar
{

#define PATCH_SIZE      31 
#define PYR_LEVELS      1
#define N_VIEWS         5000
#define N_PTS_TO_FIND   400
#define N_PTS_TO_TEACH  200
#define SIZE_BLUR       13

#define N_STRUCTS       50
#define STRUCT_SIZE     11
#define SIGNATURE_SIZE  400

// default opencv parameters
// PATCH_SIZE = 31,
// DEFAULT_STRUCTS = 50,
// DEFAULT_STRUCT_SIZE = 9,
// DEFAULT_VIEWS = 5000,
// DEFAULT_SIGNATURE_SIZE = 176,
// COMPRESSION_NONE = 0,
// COMPRESSION_RANDOM_PROJ = 1,
// COMPRESSION_PCA = 2,
// DEFAULT_COMPRESSION_METHOD = COMPRESSION_NONE

// PatchGenerator(double _backgroundMin,   double _backgroundMax,
//                double _noiseRange,      bool   _randomBlur=true,
//                double _lambdaMin=0.6,   double _lambdaMax=1.5,
//                double _thetaMin=-CV_PI, double _thetaMax=CV_PI,
//                double _phiMin=-CV_PI,   double _phiMax=CV_PI );

// Calculate random parameterized affine transformation A
// A = T(patch center) * R(theta) * R(phi)' * S(lambda1, lambda2) * R(phi) * T(-pt)

FernClassifierWrapper::FernClassifierWrapper()
    : FernClassifier()
{
}

FernClassifierWrapper::FernClassifierWrapper(const FileNode &fileNode)
    : FernClassifier(fileNode)
{
}

FernClassifierWrapper::FernClassifierWrapper(const vector<vector<Point2f> > &points,
                                             const vector<Mat> &referenceImages,
                                             const vector<vector<int> > &labels,
                                             int _nclasses, int _patchSize,
                                             int _signatureSize, int _nstructs,
                                             int _structSize, int _nviews,
                                             int _compressionMethod,
                                             const PatchGenerator &patchGenerator)
    : FernClassifier(points, referenceImages, labels, _nclasses, _patchSize, _signatureSize,
                     _nstructs, _structSize, _nviews, _compressionMethod, patchGenerator)
{
}

FernClassifierWrapper::~FernClassifierWrapper()
{
}

void FernClassifierWrapper::readBinary(std::fstream &stream)
{
    clear();

    stream.read((char *)&verbose, sizeof(verbose));
    stream.read((char *)&nstructs, sizeof(nstructs));
    stream.read((char *)&structSize, sizeof(structSize));
    stream.read((char *)&nclasses, sizeof(nclasses));
    stream.read((char *)&signatureSize, sizeof(signatureSize));
    stream.read((char *)&compressionMethod, sizeof(compressionMethod));
    stream.read((char *)&leavesPerStruct, sizeof(leavesPerStruct));
    stream.read((char *)&patchSize.width, sizeof(patchSize.width));
    stream.read((char *)&patchSize.height, sizeof(patchSize.height));

    std::vector<Feature>::size_type featuresSize;
    stream.read((char *)&featuresSize, sizeof(featuresSize));
    features.reserve(featuresSize);
    unsigned int featuresValue;
    Feature value;
    for (std::vector<Feature>::size_type i = 0; i < featuresSize; ++i) {
        stream.read((char *)&featuresValue, sizeof(featuresValue));
        value.x1 = (uchar)(featuresValue % patchSize.width);
        value.y1 = (uchar)(featuresValue / patchSize.width);
        stream.read((char *)&featuresValue, sizeof(featuresValue));
        value.x2 = (uchar)(featuresValue % patchSize.width);
        value.y2 = (uchar)(featuresValue / patchSize.width);
        features.push_back(value);
    }

    // don't read classCounters
    /*
    std::vector<int>::size_type classCountersSize;
    stream.read((char *)&classCountersSize, sizeof(classCountersSize));
    classCounters.reserve(classCountersSize);
    int classCountersValue;
    for (std::vector<int>::size_type i = 0; i < classCountersSize; ++i) {
        stream.read((char *)&classCountersValue, sizeof(classCountersValue));
        classCounters.push_back(classCountersValue);
    }
    */

    std::vector<float>::size_type posteriorsSize;
    stream.read((char *)&posteriorsSize, sizeof(posteriorsSize));
    posteriors.reserve(posteriorsSize);
    float posteriorsValue;
    for (std::vector<float>::size_type i = 0; i < posteriorsSize; ++i) {
        stream.read((char *)&posteriorsValue, sizeof(posteriorsValue));
        posteriors.push_back(posteriorsValue);
    }
}

void FernClassifierWrapper::writeBinary(std::fstream &stream) const
{
    stream.write((char *)&verbose, sizeof(verbose));
    stream.write((char *)&nstructs, sizeof(nstructs));
    stream.write((char *)&structSize, sizeof(structSize));
    stream.write((char *)&nclasses, sizeof(nclasses));
    stream.write((char *)&signatureSize, sizeof(signatureSize));
    stream.write((char *)&compressionMethod, sizeof(compressionMethod));
    stream.write((char *)&leavesPerStruct, sizeof(leavesPerStruct));
    stream.write((char *)&patchSize.width, sizeof(patchSize.width));
    stream.write((char *)&patchSize.height, sizeof(patchSize.height));

    std::vector<Feature>::size_type featuresSize = features.size();
    stream.write((char *)&featuresSize, sizeof(featuresSize));
    unsigned int featuresValue;
    for (std::vector<Feature>::const_iterator itr = features.begin(); itr != features.end(); ++itr) {
        featuresValue = itr->y1 * patchSize.width + itr->x1;
        stream.write((char *)&featuresValue, sizeof(featuresValue));
        featuresValue = itr->y2 * patchSize.width + itr->x2;
        stream.write((char *)&featuresValue, sizeof(featuresValue));
    }

    // don't write classCounters
    /*
    std::vector<int>::size_type classCountersSize = classCounters.size();
    stream.write((char *)&classCountersSize, sizeof(classCountersSize));
    for (std::vector<int>::const_iterator itr = classCounters.begin(); itr != classCounters.end(); ++itr) {
        stream.write((char *)&*itr, sizeof(*itr));
    }
    */
    
    std::vector<float>::size_type posteriorsSize = posteriors.size();
    stream.write((char *)&posteriorsSize, sizeof(posteriorsSize));
    for (std::vector<float>::const_iterator itr = posteriors.begin(); itr != posteriors.end(); ++itr) {
        stream.write((char *)&*itr, sizeof(*itr));
    }
}

FernImageDetector::FernImageDetector(const bool visualize)
    : mPatchGenerator(0, 256, 13, true, /*0.25*/0.10, 1.0/*0.6, 1.5*/, -CV_PI*1.0, CV_PI*1.0, -CV_PI*0.0, CV_PI*0.0/*-2*CV_PI, 2*CV_PI*/) // TODO: check angle values, cant be -2pi..2pi ?
    , mLDetector(3, 20, PYR_LEVELS, N_VIEWS, PATCH_SIZE, 2)
    , mClassifier()
    , mKeyPoints()
    , mImagePoints()
    , mModelPoints()
    , mVisualize(visualize)
    , mObjects()
    , mSize()
    , mCorrespondences()
    , mHomography()
    , mInlierRatio(0)
{
	//mHomography.eye(3, 3, CV_64F);
	mClassifier.resize(1);
}

FernImageDetector::~FernImageDetector()
{
}

void FernImageDetector::imagePoints(vector<CvPoint2D64f> &points)
{
	points.clear();
	for(size_t i = 0; i < mImagePoints.size(); ++i) {
		points.push_back(cvPoint2D64f(mImagePoints[i].x, mImagePoints[i].y));
	}
}

void FernImageDetector::modelPoints(vector<CvPoint3D64f> &points, bool normalize)
{
	points.clear();
	//int minx = 1e10, miny = 1e10;
	//int maxx = 0, maxy = 0;
	for(size_t i = 0; i < mModelPoints.size(); ++i) {
        CvPoint3D64f pt = cvPoint3D64f(mModelPoints[i].x, mModelPoints[i].y, 0.0);
		if(normalize) {
			//minx = (pt.x<minx)?pt.x:minx; 
			//miny = (pt.y<miny)?pt.y:miny;
			//maxx = (pt.x>maxx)?pt.x:maxx;
			//maxy = (pt.y>maxy)?pt.y:maxy;
			pt.x -= mSize.width/2;
			pt.y -= mSize.height/2;
			pt.x /= mSize.width*0.10;
			pt.y /= mSize.width*0.10;
		}
		points.push_back(pt);
	}
}

cv::Size FernImageDetector::size()
{
	return mSize;
}

cv::Mat FernImageDetector::homography()
{
	return mHomography;
}

double FernImageDetector::inlierRatio()
{
	return mInlierRatio;
}

void FernImageDetector::train(const std::string &filename)
{
	Mat object = imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	train(object);
}

void FernImageDetector::train(Mat &object)
{
	mObjects.push_back(object.clone());

    Mat blurredObject;
    GaussianBlur(mObjects[0], blurredObject, Size(SIZE_BLUR, SIZE_BLUR), 0, 0);

    if(mVisualize) {
		cvNamedWindow("Object", 1);
		imshow("Object", blurredObject);
		cv::waitKey(2000);
	}

    //buildPyramid(object, objpyr, mLDetector.nOctaves-1);
	//mLDetector(mObjects[0], mKeyPoints, N_PTS_TO_TEACH); // TODO: find robust features, TODO: in pyramids?
	mLDetector.getMostStable2D(blurredObject, mKeyPoints, N_PTS_TO_TEACH, mPatchGenerator);

	if(mVisualize) {
		for(int i = 0; i < (int)mKeyPoints.size(); ++i)
			circle(blurredObject, mKeyPoints[i].pt, int(mKeyPoints[i].size/10), CV_RGB(64,64,64));

		imshow("Object", blurredObject);
		cv::waitKey(2000);
	}

	mClassifier[0].trainFromSingleView(blurredObject,
									mKeyPoints,
									PATCH_SIZE, 
									SIGNATURE_SIZE,
									N_STRUCTS, // TODO: why was (int)mKeyPoints.size(), use here? why not a constant?
									STRUCT_SIZE,
									N_VIEWS,
									FernClassifier::COMPRESSION_NONE,
									mPatchGenerator);
	
	mSize = cv::Size(object.cols, object.rows);
}

void FernImageDetector::findFeatures(Mat &object, bool planeAssumption)
{
	//cv::flip(object, object, 1);

	vector<KeyPoint> keypoints;
	vector<Mat> objpyr;

	GaussianBlur(object, object, Size(SIZE_BLUR, SIZE_BLUR), 0, 0);
	//buildPyramid(object, objpyr, mLDetector.nOctaves-1);
	mLDetector.nOctaves = 1;
	mLDetector(object/*objpyr*/, keypoints, N_PTS_TO_FIND);
		
	int m = mKeyPoints.size();
	int n = keypoints.size();
    vector<int> bestMatches(m, -1);
    vector<float> maxLogProb(m, -FLT_MAX);
    vector<float> signature;
	vector<int> pairs;

	for(size_t i = 0; i < keypoints.size(); ++i) {
		Point2f pt = keypoints[i].pt;
		//int oct = keypoints[i].octave; std::cout<<"oct "<<oct<<std::endl;
		int k = mClassifier[0](object /*objpyr[oct]*/, pt, signature);
		if(k >= 0 && (bestMatches[k] < 0 || signature[k] > maxLogProb[k])) {
			maxLogProb[k] = signature[k];
			bestMatches[k] = i;
		}
	}

	for(int i = 0; i < m; i++ )
		if(bestMatches[i] >= 0) {
			pairs.push_back(i);
			pairs.push_back(bestMatches[i]);
    }

        mCorrespondences = Mat(mObjects[0].rows + object.rows, std::max( mObjects[0].cols, object.cols), CV_8UC3);
		mCorrespondences = Scalar(0.);
        Mat part(mCorrespondences, Rect(0, 0, mObjects[0].cols, mObjects[0].rows));
        cvtColor(mObjects[0], part, CV_GRAY2BGR);
		part = Mat(mCorrespondences, Rect(0, mObjects[0].rows, object.cols, object.rows));
		cvtColor(object, part, CV_GRAY2BGR);

		for(int i = 0; i < (int)keypoints.size(); ++i)
			circle(object, keypoints[i].pt, int(keypoints[i].size/5), CV_RGB(64,64,64));

		vector<Point2f> fromPt, toPt;
		vector<uchar> mask;
		for(int i = 0; i < m; ++i)
			if( bestMatches[i] >= 0 ){
				fromPt.push_back(mKeyPoints[i].pt);
				toPt.push_back(keypoints[bestMatches[i]].pt);
			}

			static double valmin = 1.0;
			static double valmax = 0.0;
			mModelPoints.clear();
			mImagePoints.clear();
			int n_inliers = 0;

			if(planeAssumption && fromPt.size() > 8) {
				cv::Mat H = cv::findHomography(Mat(fromPt), Mat(toPt), mask, RANSAC/*CV_LMEDS*/, 20);
				mHomography = H;
				//CompareModelAndObservation();

				for(size_t i = 0, j = 0; i < (int)pairs.size(); i += 2, ++j) {
					if(mask[j]) {
						cv::Point2f pi(keypoints[pairs[i+1]].pt);
						cv::Point2f pw(mKeyPoints[pairs[i]].pt);
						mModelPoints.push_back(pw);
						mImagePoints.push_back(pi);
						line(mCorrespondences, mKeyPoints[pairs[i]].pt,
							keypoints[pairs[i+1]].pt + Point2f(0.0,(float)mObjects[0].rows),
							Scalar(i*i%244,100-i*100%30,i*i-50*i));
						n_inliers++;
					}
				}
			} else {
				for(size_t i = 0, j = 0; i < (int)pairs.size(); i += 2, ++j) {
					cv::Point2f pi(keypoints[pairs[i+1]].pt);
					cv::Point2f pw(mKeyPoints[pairs[i]].pt);
					mModelPoints.push_back(pw);
					mImagePoints.push_back(pi);
					line(mCorrespondences, mKeyPoints[pairs[i]].pt,
						keypoints[pairs[i+1]].pt + Point2f(0.0,(float)mObjects[0].rows),
						Scalar(i*i%244,100-i*100%30,i*i-50*i));
				}
			}
			

			double val = 0.0;
			if(fromPt.size()>0) val = 1.*n_inliers/fromPt.size();
			if(val > valmax) valmax = val;
			if(val < valmin) valmin = val;

			mInlierRatio = val;

    if (mVisualize) {
        cvNamedWindow("Matches", 1);
        imshow("Matches", mCorrespondences);
        cv::waitKey(1);
    }
}

bool FernImageDetector::read(const std::string &filename, const bool binary)
{
    if (binary) {
        std::fstream bs(filename.c_str(), std::fstream::in | std::fstream::binary);

        if (!bs.is_open()) {
            return false;
        }

        bs.read((char *)&mLDetector.radius, sizeof(mLDetector.radius));
        bs.read((char *)&mLDetector.threshold, sizeof(mLDetector.threshold));
        bs.read((char *)&mLDetector.nOctaves, sizeof(mLDetector.nOctaves));
        bs.read((char *)&mLDetector.nViews, sizeof(mLDetector.nViews));
        bs.read((char *)&mLDetector.verbose, sizeof(mLDetector.verbose));
        bs.read((char *)&mLDetector.baseFeatureSize, sizeof(mLDetector.baseFeatureSize));
        bs.read((char *)&mLDetector.clusteringDistance, sizeof(mLDetector.clusteringDistance));

        mClassifier[0].readBinary(bs);

        std::vector<float>::size_type size;
        bs.read((char *)&size, sizeof(size));
        mKeyPoints.reserve(size);
        KeyPoint value;
        for (std::vector<float>::size_type i = 0; i < size; ++i) {
            bs.read((char *)&value.pt.x, sizeof(value.pt.x));
            bs.read((char *)&value.pt.y, sizeof(value.pt.y));
            bs.read((char *)&value.size, sizeof(value.size));
            bs.read((char *)&value.angle, sizeof(value.angle));
            bs.read((char *)&value.response, sizeof(value.response));
            bs.read((char *)&value.octave, sizeof(value.octave));
            bs.read((char *)&value.class_id, sizeof(value.class_id));
            mKeyPoints.push_back(value);
        }

        bs.read((char *)&mSize.width, sizeof(mSize.width));
        bs.read((char *)&mSize.height, sizeof(mSize.height));

        std::vector<Mat>::size_type objectsSize;
        bs.read((char *)&objectsSize, sizeof(objectsSize));
        mObjects.reserve(objectsSize);
        int rows;
        int cols;
        int type;
        for (std::vector<Mat>::size_type i = 0; i < objectsSize; ++i) {
            bs.read((char *)&rows, sizeof(rows));
            bs.read((char *)&cols, sizeof(cols));
            bs.read((char *)&type, sizeof(type));
            Mat objectsValue(rows, cols, type);
            bs.read((char *)objectsValue.data, objectsValue.elemSize() * objectsValue.total());
            mObjects.push_back(objectsValue);
        }

        bs.close();
    }
    else {
        FileStorage fs(filename, FileStorage::READ);

        if (!fs.isOpened()) {
            return false;
        }

        FileNode node = fs.getFirstTopLevelNode();
        std::cout << "loaded file" << std::endl;
        cv::read(node["model_points"], mKeyPoints);
        std::cout << "loaded model points" << std::endl;
        mClassifier[0].read(node["fern_classifier"]);
        std::cout << "loaded classifier" << std::endl;
    }

    return true;
}

bool FernImageDetector::write(const std::string &filename, const bool binary)
{
    if (binary) {
        std::fstream bs(filename.c_str(), std::fstream::out | std::fstream::binary);

        if (!bs.is_open()) {
            return false;
        }

        bs.write((char *)&mLDetector.radius, sizeof(mLDetector.radius));
        bs.write((char *)&mLDetector.threshold, sizeof(mLDetector.threshold));
        bs.write((char *)&mLDetector.nOctaves, sizeof(mLDetector.nOctaves));
        bs.write((char *)&mLDetector.nViews, sizeof(mLDetector.nViews));
        bs.write((char *)&mLDetector.verbose, sizeof(mLDetector.verbose));
        bs.write((char *)&mLDetector.baseFeatureSize, sizeof(mLDetector.baseFeatureSize));
        bs.write((char *)&mLDetector.clusteringDistance, sizeof(mLDetector.clusteringDistance));

        mClassifier[0].writeBinary(bs);

        std::vector<float>::size_type size = mKeyPoints.size();
        bs.write((char *)&size, sizeof(size));
        for (std::vector<KeyPoint>::const_iterator itr = mKeyPoints.begin(); itr != mKeyPoints.end(); ++itr) {
            bs.write((char *)&itr->pt.x, sizeof(itr->pt.x));
            bs.write((char *)&itr->pt.y, sizeof(itr->pt.y));
            bs.write((char *)&itr->size, sizeof(itr->size));
            bs.write((char *)&itr->angle, sizeof(itr->angle));
            bs.write((char *)&itr->response, sizeof(itr->response));
            bs.write((char *)&itr->octave, sizeof(itr->octave));
            bs.write((char *)&itr->class_id, sizeof(itr->class_id));
        }

        bs.write((char *)&mSize.width, sizeof(mSize.width));
        bs.write((char *)&mSize.height, sizeof(mSize.height));

        std::vector<Mat>::size_type objectsSize = mObjects.size();
        bs.write((char *)&objectsSize, sizeof(objectsSize));
        for (std::vector<Mat>::const_iterator itr = mObjects.begin(); itr != mObjects.end(); ++itr) {
            bs.write((char *)&itr->rows, sizeof(itr->rows));
            bs.write((char *)&itr->cols, sizeof(itr->cols));
            int type = itr->type();
            bs.write((char *)&type, sizeof(type));
            bs.write((char *)itr->data, itr->elemSize() * itr->total());
        }

        bs.close();
    }
    else {
        FileStorage fs(filename, FileStorage::WRITE);

        if (!fs.isOpened()) {
            return false;
        }

        WriteStructContext ws(fs, "fern_image_detector", CV_NODE_MAP);
        cv::write(fs, "model_points", mKeyPoints);
        mClassifier[0].write(fs, "fern_classifier");
    }

    return true;
}

} // namespace alvar

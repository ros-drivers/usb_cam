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

#include "ConnectedComponents.h"
#include "Draw.h"
#include <cassert>

using namespace std;

namespace alvar {
using namespace std;

Labeling::Labeling()
{
	gray = 0;
	bw	 = 0;
	cam  = 0;
	thresh_param1 = 31;
	thresh_param2 = 5;
}

Labeling::~Labeling()
{
	if(gray)
		cvReleaseImage(&gray);
	if(bw)
		cvReleaseImage(&bw);
}

bool Labeling::CheckBorder(CvSeq* contour, int width, int height)
{
	bool ret = true;
	for(int i = 0; i < contour->total; ++i)
	{
		CvPoint* pt = (CvPoint*)cvGetSeqElem(contour, i);
		if((pt->x <= 1) || (pt->x >= width-2) || (pt->y <= 1) || (pt->y >= height-2)) ret = false;
	}
	return ret;
}

LabelingCvSeq::LabelingCvSeq() : _n_blobs(0), _min_edge(20), _min_area(25)
{
	SetOptions();
	storage = cvCreateMemStorage(0);
}

LabelingCvSeq::~LabelingCvSeq()
{
	if(storage)
		cvReleaseMemStorage(&storage);
}

void LabelingCvSeq::SetOptions(bool _detect_pose_grayscale) {
    detect_pose_grayscale = _detect_pose_grayscale;
}

void LabelingCvSeq::LabelSquares(IplImage* image, bool visualize)
{

    if (gray && ((gray->width != image->width) || (gray->height != image->height))) {
        cvReleaseImage(&gray); gray=NULL;
        if (bw) cvReleaseImage(&bw); bw=NULL;
    }
    if (gray == NULL) {
        gray = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
        gray->origin = image->origin;
        bw = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
        bw->origin = image->origin;
    }

    // Convert grayscale and threshold
    if(image->nChannels == 4)
        cvCvtColor(image, gray, CV_RGBA2GRAY);
    else if(image->nChannels == 3)
        cvCvtColor(image, gray, CV_RGB2GRAY);
    else if(image->nChannels == 1)
        cvCopy(image, gray);
    else {
        cerr<<"Unsupported image format"<<endl;
    }

    cvAdaptiveThreshold(gray, bw, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, thresh_param1, thresh_param2);
    //cvThreshold(gray, bw, 127, 255, CV_THRESH_BINARY_INV);

    CvSeq* contours;
    CvSeq* squares = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSeq), storage);
    CvSeq* square_contours = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSeq), storage);

    cvFindContours(bw, storage, &contours, sizeof(CvContour),
        CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0,0));

    while(contours)
    {
        if(contours->total < _min_edge)
        {
            contours = contours->h_next;
            continue;
        }

        CvSeq* result = cvApproxPoly(contours, sizeof(CvContour), storage,
                                     CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.035, 0 ); // TODO: Parameters?

        if( result->total == 4 && CheckBorder(result, image->width, image->height) && 
            fabs(cvContourArea(result,CV_WHOLE_SEQ)) > _min_area && // TODO check limits
            cvCheckContourConvexity(result) ) // ttehop: Changed to 'contours' instead of 'result'
        {
                cvSeqPush(squares, result);
                cvSeqPush(square_contours, contours);
        }
        contours = contours->h_next;
    }

    _n_blobs = squares->total;
    blob_corners.resize(_n_blobs);

    // For every detected 4-corner blob
    for(int i = 0; i < _n_blobs; ++i)
    {
        vector<Line> fitted_lines(4);
        blob_corners[i].resize(4);
        CvSeq* sq = (CvSeq*)cvGetSeqElem(squares, i);
        CvSeq* square_contour = (CvSeq*)cvGetSeqElem(square_contours, i);
        
        for(int j = 0; j < 4; ++j)
        {
            CvPoint* pt0 = (CvPoint*)cvGetSeqElem(sq, j);
            CvPoint* pt1 = (CvPoint*)cvGetSeqElem(sq, (j+1)%4);
            int k0=-1, k1=-1;
            for (int k = 0; k<square_contour->total; k++) {
                CvPoint* pt2 = (CvPoint*)cvGetSeqElem(square_contour, k);
                if ((pt0->x == pt2->x) && (pt0->y == pt2->y)) k0=k;
                if ((pt1->x == pt2->x) && (pt1->y == pt2->y)) k1=k;
            }
            int len;
            if (k1 >= k0) len = k1-k0-1; // neither k0 nor k1 are included
            else len = square_contour->total-k0+k1-1;
            if (len == 0) len = 1;

            CvMat* line_data = cvCreateMat(1, len, CV_32FC2);
            for (int l=0; l<len; l++) {
                int ll = (k0+l+1)%square_contour->total;
                CvPoint* p = (CvPoint*)cvGetSeqElem(square_contour, ll);
                CvPoint2D32f pp;
                pp.x = float(p->x);
                pp.y = float(p->y);

                // Undistort
                if(cam)
                    cam->Undistort(pp);

                CV_MAT_ELEM(*line_data, CvPoint2D32f, 0, l) = pp;
            }

            // Fit edge and put to vector of edges
            float params[4] = {0};

            // TODO: The detect_pose_grayscale is still under work...
            /*
            if (detect_pose_grayscale &&
                (pt0->x > 3) && (pt0->y > 3) &&
                (pt0->x < (gray->width-4)) &&
                (pt0->y < (gray->height-4)))
            {
                // ttehop: Grayscale experiment
                FitLineGray(line_data, params, gray);
            }
            */
            cvFitLine(line_data, CV_DIST_L2, 0, 0.01, 0.01, params);

            //cvFitLine(line_data, CV_DIST_L2, 0, 0.01, 0.01, params);
            ////cvFitLine(line_data, CV_DIST_HUBER, 0, 0.01, 0.01, params);
            Line line = Line(params);
            if(visualize) DrawLine(image, line);
            fitted_lines[j] = line;

            cvReleaseMat(&line_data);
        }

        // Calculated four intersection points
        for(size_t j = 0; j < 4; ++j)
        {
            PointDouble intc = Intersection(fitted_lines[j],fitted_lines[(j+1)%4]);

            // TODO: Instead, test OpenCV find corner in sub-pix...
            //CvPoint2D32f pt = cvPoint2D32f(intc.x, intc.y);
            //cvFindCornerSubPix(gray, &pt,
            //                   1, cvSize(3,3), cvSize(-1,-1),
            //                   cvTermCriteria(
            //                   CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,10,1e-4));
            
            // TODO: Now there is a wierd systematic 0.5 pixel error that is fixed here...
            //intc.x += 0.5;
            //intc.y += 0.5;

            if(cam) cam->Distort(intc);

            // TODO: Should we make this always counter-clockwise or clockwise?
            /*
            if (image->origin && j == 1) blob_corners[i][3] = intc;
            else if (image->origin && j == 3) blob_corners[i][1] = intc;
            else blob_corners[i][j] = intc;
            */
            blob_corners[i][j] = intc;
        }
        if (visualize) {
            for(size_t j = 0; j < 4; ++j) {
                PointDouble &intc = blob_corners[i][j];
                if (j == 0) cvCircle(image, cvPoint(int(intc.x), int(intc.y)), 5, CV_RGB(255, 255, 255));
                if (j == 1) cvCircle(image, cvPoint(int(intc.x), int(intc.y)), 5, CV_RGB(255, 0, 0));
                if (j == 2) cvCircle(image, cvPoint(int(intc.x), int(intc.y)), 5, CV_RGB(0, 255, 0));
                if (j == 3) cvCircle(image, cvPoint(int(intc.x), int(intc.y)), 5, CV_RGB(0, 0, 255));
            }
        }
    }

    cvClearMemStorage(storage);
}

CvSeq* LabelingCvSeq::LabelImage(IplImage* image, int min_size, bool approx)
{
	assert(image->origin == 0); // Currently only top-left origin supported
	if (gray && ((gray->width != image->width) || (gray->height != image->height))) {
		cvReleaseImage(&gray); gray=NULL;
		if (bw) cvReleaseImage(&bw); bw=NULL;
	}
	if (gray == NULL) {
		gray = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
		gray->origin = image->origin;
		bw = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
		bw->origin = image->origin;
	}

	// Convert grayscale and threshold
	if(image->nChannels == 4)
		cvCvtColor(image, gray, CV_RGBA2GRAY);
	else if(image->nChannels == 3)
		cvCvtColor(image, gray, CV_RGB2GRAY);
	else if(image->nChannels == 1)
		cvCopy(image, gray);
	else {
		cerr<<"Unsupported image format"<<endl;
	}

	cvAdaptiveThreshold(gray, bw, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, thresh_param1, thresh_param2);

	CvSeq* contours;
	CvSeq* edges = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSeq), storage);
	CvSeq* squares = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSeq), storage);

	cvFindContours(bw, storage, &contours, sizeof(CvContour),
		CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
	//cvFindContours(bw, storage, &contours, sizeof(CvContour),
	//	CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));


	while(contours)
	{
		if(contours->total < min_size)
		{
			contours = contours->h_next;
			continue;
		}
		
		if(approx)
		{
			CvSeq* result = cvApproxPoly(contours, sizeof(CvContour), storage,
										CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 ); // TODO: Parameters?

			if(cvCheckContourConvexity(result))
			{
					cvSeqPush(squares, result);
			}
		}	
		else
			cvSeqPush(squares, contours);
	
		contours = contours->h_next;
	}

	cvClearMemStorage(storage);

	return squares;
}

inline int round(double x) {
	return (x)>=0?(int)((x)+0.5):(int)((x)-0.5);
}

template<class T>
inline T absdiff(T c1, T c2) {
	return (c2>c1?c2-c1:c1-c2);
}

//#define SHOW_DEBUG
#ifdef SHOW_DEBUG
#include "highgui.h"
#endif

// TODO: This should be in LabelingCvSeq ???
void FitLineGray(CvMat *line_data, float params[4], IplImage *gray) {
	// this very simple approach works...
	/*
	float *cx = &(params[2]);
	float *cy = &(params[3]);
	float *sx = &(params[0]);
	float *sy = &(params[1]);
	CvPoint2D32f *p1 = (CvPoint2D32f*)CV_MAT_ELEM_PTR_FAST(*line_data, 0, 0, sizeof(CvPoint2D32f));
	CvPoint2D32f *p2 = (CvPoint2D32f*)CV_MAT_ELEM_PTR_FAST(*line_data, 0, line_data->cols-1, sizeof(CvPoint2D32f));
	*cx = p1->x; *cy = p1->y;
	*sx = p2->x - p1->x; *sy = p2->y - p1->y;
	return;
	*/

#ifdef SHOW_DEBUG
	IplImage *tmp = cvCreateImage(cvSize(gray->width, gray->height), IPL_DEPTH_8U, 3);
	IplImage *tmp2 = cvCreateImage(cvSize(gray->width*5, gray->height*5), IPL_DEPTH_8U, 3);
	cvCvtColor(gray, tmp, CV_GRAY2RGB);
	cvResize(tmp, tmp2, CV_INTER_NN);
#endif

	// Discover 1st the line normal direction
	CvPoint2D32f *p1 = (CvPoint2D32f*)CV_MAT_ELEM_PTR_FAST(*line_data, 0, 0, sizeof(CvPoint2D32f));
	CvPoint2D32f *p2 = (CvPoint2D32f*)CV_MAT_ELEM_PTR_FAST(*line_data, 0, line_data->cols-1, sizeof(CvPoint2D32f));
	double dx = +(p2->y - p1->y);
	double dy = -(p2->x - p1->x);
	if ((dx == 0) && (dy == 0)) return;
	else if      (dx == 0) { dy /= dy; }
	else if (dy == 0) { dx /= dx; }
	else if (abs(dx) > abs(dy)) { dy /= dx; dx /= dx; }
	else              { dx /= dy; dy /= dy; }

	// Build normal search table
	const int win_size=5;
	const int win_mid=win_size/2;
	const int diff_win_size=win_size-1;
	double xx[win_size], yy[win_size];
	double dxx[diff_win_size], dyy[diff_win_size];
	xx[win_mid] = 0; yy[win_mid] = 0;
	for (int i=1; i<=win_size/2; i++) {
		xx[win_mid + i] = round(i*dx);
		xx[win_mid - i] = -xx[win_mid + i];
		yy[win_mid + i] = round(i*dy);
		yy[win_mid - i] = -yy[win_mid + i];
	}
	for (int i=0; i<diff_win_size; i++) {
		dxx[i] = (xx[i]+xx[i+1])/2;
		dyy[i] = (yy[i]+yy[i+1])/2;
	}

	// Adjust the points
	for (int l=0; l<line_data->cols; l++) {
		CvPoint2D32f *p = (CvPoint2D32f*)CV_MAT_ELEM_PTR_FAST(*line_data, 0, l, sizeof(CvPoint2D32f));

		double dx=0, dy=0, ww=0;
		for (int i=0; i<diff_win_size; i++) {
			unsigned char c1 = (unsigned char)gray->imageData[int((p->y+yy[i])*gray->widthStep+(p->x+xx[i]))];
			unsigned char c2 = (unsigned char)gray->imageData[int((p->y+yy[i+1])*gray->widthStep+(p->x+xx[i+1]))];
#ifdef SHOW_DEBUG
			cvCircle(tmp2, cvPoint((p->x+xx[i])*5+2,(p->y+yy[i])*5+2), 0, CV_RGB(0,0,255));
			cvCircle(tmp2, cvPoint((p->x+xx[i+1])*5+2,(p->y+yy[i+1])*5+2), 0, CV_RGB(0,0,255));
#endif
			double w = absdiff(c1, c2);
			dx += dxx[i]*w;
			dy += dyy[i]*w;
			ww += w;
		}
		if (ww > 0) {
			dx /= ww; dy /= ww;
		}
#ifdef SHOW_DEBUG
		cvLine(tmp2, cvPoint(p->x*5+2,p->y*5+2), cvPoint((p->x+dx)*5+2, (p->y+dy)*5+2), CV_RGB(0,255,0));
		p->x += float(dx); p->y += float(dy);
		cvCircle(tmp2, cvPoint(p->x*5+2,p->y*5+2), 0, CV_RGB(255,0,0));
#else
		p->x += float(dx); p->y += float(dy);
#endif
	}

#ifdef SHOW_DEBUG
	cvNamedWindow("tmp");
	cvShowImage("tmp",tmp2);
	cvWaitKey(0);
	cvReleaseImage(&tmp);
	cvReleaseImage(&tmp2);
#endif
}

} // namespace alvar

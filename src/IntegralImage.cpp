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

#include "IntegralImage.h"

namespace alvar {

void IntIndex::update_next_step() {
    next_step = step;
    estep += step_remainder;
    if (estep >= steps) {
        estep -= steps;
        next_step++;
    }
}

IntIndex::IntIndex(int _res, int _steps) {
    res = _res;
    steps = _steps;
    operator=(0);
}
int IntIndex::operator=(int v) {
    index = 0;
    step = res / steps;
    step_remainder = res % steps;
    estep = 0;
    update_next_step();
    while ((index+next_step-1) < v) next();
    return index;
}
int IntIndex::next() {
    index += next_step;
    update_next_step();
    return index;
}
int IntIndex::get() const {
    return index;
}
int IntIndex::get_next_step() const {
    return next_step;
}
int IntIndex::end() const {
    return res;
}

IntegralImage::IntegralImage() {
	sum = 0;
}
IntegralImage::~IntegralImage() {
	if (sum) cvReleaseImage(&sum);
}
void IntegralImage::Update(IplImage *gray) {
	if ((sum == 0) ||
		(sum->height != gray->width+1) ||
		(sum->width != gray->height+1))
	{
		if (sum) cvReleaseImage(&sum);
		// TODO: Now we assume 'double' - is it ok?
		sum = cvCreateImage(cvSize(gray->width+1, gray->height+1), IPL_DEPTH_64F, 1);
	}
	cvIntegral(gray, sum);
}
double IntegralImage::GetSum(CvRect &rect, int *count /*=0*/) {
	int x1 = rect.x;
	int x2 = rect.x + rect.width; // Note, not -1
	int y1 = rect.y;
	int y2 = rect.y + rect.height;
	//cout<<x1<<","<<y1<<"-"<<x2<<","<<y2<<endl;
	/*
	double v = +cvGet2D(sum, y2, x2).val[0]
	           -cvGet2D(sum, y2, x1).val[0]
			   -cvGet2D(sum, y1, x2).val[0]
			   +cvGet2D(sum, y1, x1).val[0];
    */
	double v = +((double *)sum->imageData)[y2*sum->width+x2]
	           -((double *)sum->imageData)[y2*sum->width+x1]
	           -((double *)sum->imageData)[y1*sum->width+x2]
	           +((double *)sum->imageData)[y1*sum->width+x1];

	if (count) *count = rect.width*rect.height;
	return v;
}
double IntegralImage::GetAve(CvRect &rect) {
	int count=1;
	return GetSum(rect, &count)/count;
}
void IntegralImage::GetSubimage(const CvRect &rect, IplImage *sub) {
    int yi=0;
    for (IntIndex yy(rect.height, sub->height); yy.get() != yy.end(); yy.next(),yi++) {
	    int xi=0;
	    for (IntIndex xx(rect.width, sub->width); xx.get() != xx.end(); xx.next(),xi++) {
			//cout<<"res: "<<sum->height<<","<<sum->width<<" - ";
    	    //cout<<xi<<","<<yi<<": "<<rect.x<<","<<rect.y<<": "<<xx.get()<<","<<yy.get()<<endl;
			CvRect r = {
				rect.x+xx.get(),
				rect.y+yy.get(),
				xx.get_next_step(),
				yy.get_next_step()
			};
            double ave = GetAve(r);

            //cvSet2D(sub, yi, xi, cvScalar(ave));
			// TODO: Now we assume 8-bit gray
            sub->imageData[yi*sub->widthStep+xi] = (char)ave;
        }
	}
}
void IntegralGradient::CalculatePointNormals(IplImage *gray) {
	int width = gray->width-1;
	int height = gray->height-1;
	if ((normalx == 0) || 
		(normalx->width != width) ||
		(normalx->height != height))
	{
		if (normalx) cvReleaseImage(&normalx);
		if (normaly) cvReleaseImage(&normaly);
		normalx = cvCreateImage(cvSize(width, height), IPL_DEPTH_64F, 1);
		normaly = cvCreateImage(cvSize(width, height), IPL_DEPTH_64F, 1);
	}
    for (int j=0; j<height; j++) {
        for (int i=0; i<width; i++) {
			/*
            // As we assume top-left coordinates we have these reverse compared to Donahue1992
            double a4 = cvGet2D(gray, j, i+1).val[0];
            double a3 = cvGet2D(gray, j, i).val[0];
            double a2 = cvGet2D(gray, j+1, i).val[0];
            double a1 = cvGet2D(gray, j+1, i+1).val[0];
            // Normal vectors;
            double nx = (-a1+a2+a3-a4)/4; 
            double ny = (-a1-a2+a3+a4)/4;
            cvSet2D(normalx, j, i, cvScalar(nx));
            cvSet2D(normaly, j, i, cvScalar(ny));
			*/
            // As we assume top-left coordinates we have these reverse compared to Donahue1992
			// TODO: Now we assume 8-bit gray
            double a4 = (unsigned char)gray->imageData[(j)*gray->widthStep+(i+1)];
            double a3 = (unsigned char)gray->imageData[(j)*gray->widthStep+(i)];
            double a2 = (unsigned char)gray->imageData[(j+1)*gray->widthStep+(i)];
            double a1 = (unsigned char)gray->imageData[(j+1)*gray->widthStep+(i+1)];
            // Normal vectors;
            double nx = (-a1+a2+a3-a4)/4; 
            double ny = (-a1-a2+a3+a4)/4;
			((double *)normalx->imageData)[j*normalx->width+i] = nx;
			((double *)normaly->imageData)[j*normaly->width+i] = ny;
        }
    } 
}
IntegralGradient::IntegralGradient() {
	normalx = 0;
	normaly = 0;
}
IntegralGradient::~IntegralGradient() {
	if (normalx) cvReleaseImage(&normalx);
	if (normaly) cvReleaseImage(&normaly);
}
void IntegralGradient::Update(IplImage *gray) {
	CalculatePointNormals(gray);
	integx.Update(normalx);
	integy.Update(normaly);
}
void IntegralGradient::GetGradient(CvRect &rect, double *dirx, double *diry, int *count /*=0*/) {
	CvRect r = {rect.x, rect.y, rect.width-1, rect.height-1};
	if (count) *dirx = integx.GetSum(r, count);
	else *dirx = integx.GetSum(r);
	*diry = integy.GetSum(r);
}
void IntegralGradient::GetAveGradient(CvRect &rect, double *dirx, double *diry) {
	int count=1;
	GetGradient(rect, dirx, diry, &count);
	*dirx /= count;
	*diry /= count;
}

} // namespace alvar

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

#include "Line.h"

using namespace std;

namespace alvar {
using namespace std;

Line::Line(float params[4])
{
	c.x = params[2];
	c.y = params[3];
	s.x = params[0];
	s.y = params[1];
}

void FitLines(vector<Line>& lines)
{

}

int FitLines(vector<Line> &lines,
			const vector<int>& corners,
			const vector<PointInt >& edge,
			IplImage *grey /*=0*/) // grey image for future sub pixel accuracy
{
	lines.clear();
	for(unsigned j = 0; j < corners.size(); ++j)
	{
		int start, end, first;
		int size = (int)edge.size();

		first = corners[0];
		start = corners[j];

		if(j < corners.size()-1)
			end  = corners[j+1];
		else
			end = first;

		int len = 0;

		if(start < end)
			len = end-start+1;
		else
			len = size-start+end+1;

		int ind;
		double* data = new double[2*len];

		// OpenCV routine... 
		CvMat* line_data = cvCreateMat(1, len, CV_32FC2);
		for(int i = 0; i < len; ++i)
		{
			ind = i + start;
			if(ind >= size)
			ind = ind-size;

			double px = double(edge[ind].x);
			double py = double(edge[ind].y);
			CV_MAT_ELEM(*line_data, CvPoint2D32f, 0, i) = cvPoint2D32f(px, py);
		}

		float params[4] = {0};
		cvFitLine(line_data, CV_DIST_L2, 0, 0.01, 0.01, params);
		lines.push_back(Line(params));

		delete [] data;
		cvReleaseMat(&line_data);

	}

	return lines.size();
}

PointDouble Intersection(const Line& l1, const Line& l2)
{

	double vx = l1.s.x;
	double vy = l1.s.y;
	double ux = l2.s.x;
	double uy = l2.s.y;
	double wx = l2.c.x-l1.c.x;
	double wy = l2.c.y-l1.c.y;

	double s, px, py;
	double tmp = vx*uy-vy*ux;
	if(tmp==0) tmp = 1;

	//if(/*tmp <= 1.f && tmp >= -1.f && */tmp != 0.f && ang > 0.1)
	{
		s = (vy*wx-vx*wy) / (tmp);
		px = l2.c.x+s*ux;
		py = l2.c.y+s*uy;
	}

	return PointDouble(px, py);
}

} // namespace alvar

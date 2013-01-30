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

#include "TrackerPsa.h"

using namespace std;

namespace alvar {
using namespace std;

TrackerPsa::TrackerPsa(int _max_shift) {
	max_shift = _max_shift;
	x_res = 0; y_res = 0;
	hor = 0; horprev = 0;
	ver = 0; verprev = 0;
	framecount = 0;
}

TrackerPsa::~TrackerPsa() {
	if (hor) delete [] hor;
	if (horprev) delete [] horprev;
	if (ver) delete [] ver;
	if (verprev) delete [] verprev;
}

double TrackerPsa::Track(IplImage *img) {
	long best_x_factor=99999999;
	long best_y_factor=99999999;
	long worst_x_factor=0;
	long worst_y_factor=0;
	double quality=0;

	//std::cout<<"Moi1"<<std::endl;
	// Init
	if ((x_res != img->width) || (y_res != img->height)) {
		x_res = img->width;
		y_res = img->height;
		if (hor) delete [] hor;
		if (horprev) delete [] horprev;
		if (ver) delete [] ver;
		if (verprev) delete [] verprev;
		hor = new long [x_res];
		horprev = new long [x_res];
		ver = new long [y_res];
		verprev = new long [y_res];
		framecount=0;
	}
	framecount++;

	// Make shift tables
	memset(hor, 0, sizeof(long)*x_res);
	memset(ver, 0, sizeof(long)*y_res);
	for (int y=0; y<y_res; y++) {
		for (int x=0; x<x_res; x++) {
			unsigned char c = (unsigned char)cvGet2D(img, y, x).val[0];
			hor[x] += c;
			ver[y] += c;
		}
	}

	// If this is first frame -- no motion
	if (framecount == 1) {
		xd=0; yd=0;
	} 

	// Otherwais detect for motion
	else {
		// Sequence for s: 0, 1, -1, 2, -2, ... -(max_shift-1), (max_shift-1)
		for (int s=0; s<max_shift; (s>0 ? s=-s : s=-s+1) ) {
			long factor=0;
			long factorfirst=0;
			int count=0;
			for (int x=0; x<x_res; x++) {
				int x2 = x+s;
				if ((x2 > 0) && (x2<x_res)) {
					factor += labs(hor[x2] - horprev[x]);
					count++;
				}
			}
			factor /= count;
			if (factor < best_x_factor) {
				best_x_factor=factor;
				xd=s;
			}
			if (factor > worst_x_factor) worst_x_factor=factor;
		}
		for (int s=0; s<max_shift; (s>0 ? s=-s : s=-s+1)) {
			long factor=0;
			int count=0;
			for (int y=0; y<y_res; y++) {
				int y2 = y+s;
				if ((y2 > 0) && (y2<y_res)) {
					factor += labs(ver[y2] - verprev[y]);
					count++;
				}
			}
			factor /= count;
			if (factor < best_y_factor) {
				best_y_factor=factor;
				yd=s;
			}
			if (factor > worst_y_factor) worst_y_factor=factor;
		}
		// Figure out the quality?
		// We assume the result is poor if the 
		// worst factor is very near the best factor
		int qual_x = (worst_x_factor - best_x_factor)/y_res;
		int qual_y = (worst_y_factor - best_y_factor)/x_res;
		quality = std::min(qual_x, qual_y);
	}

	// Swap memories
	long *tmp;
	tmp=hor; hor=horprev; horprev=tmp;
	tmp=ver; ver=verprev; verprev=tmp;

	// Return confidence (bigger is better)
	// Smaller than 10 is poor, and you almost certainly have 
	// problems with quality values less than 5.
	//printf("%d\n", quality);
	return quality;
}

void TrackerPsa::Compensate(double *x, double *y) {
	*x += xd; *y += yd;
}

TrackerPsaRot::TrackerPsaRot(int _max_shift) : TrackerPsa(_max_shift) {
	rotd = 0;
	rot=new double [360];
	rotprev=new double[360];
	rot_count=new int[360];
}

TrackerPsaRot::~TrackerPsaRot() {
	if (rot) delete [] rot;
	if (rotprev) delete [] rotprev;
	if (rot_count) delete [] rot_count;
}

double TrackerPsaRot::Track(IplImage *img) {
	long best_rot_factor=99999999;
	double conf1 = TrackerPsa::Track(img);

	if (framecount == 1) {
		rotd=0;
	}
	else {
		memset(rot, 0, sizeof(double)*360);
		memset(rot_count, 0, sizeof(int)*360);
		for (int y=0; y<y_res; y++) {
			for (int x=0; x<x_res; x++) {
				double y2 = y-(y_res/2)-(yd);
				double x2 = x-(x_res/2)-(xd);
				double r = sqrt((double)y2*y2 + x2*x2);
				int theta = int(atan2((double)y2, (double)x2)*180/3.14159265);
				if (theta < 0) theta+=360;
				if ((y >= 0) && (y < img->height) &&
					(x >= 0) && (x < img->width))
				{
					rot[theta] += (unsigned char)cvGet2D(img, y, x).val[0];
					rot_count[theta] ++;
				}
			}
		}
		for (int i=0; i<360; i++) {
			rot[i] /= rot_count[i];
		}
		for (int s=0; s<45; (s>0 ? s=-s : s=-s+1)) {
			long factor=0;
			for (int theta=0; theta<360; theta++) {
				int theta2 = theta+s;
				if (theta2 < 0) theta2+=360;
				if (theta2 >= 360) theta2-=360;
				factor += (long)labs(long(rot[theta2] - rotprev[theta]));
			}
			if (factor < best_rot_factor) {
				best_rot_factor=factor;
				rotd=s;
			}
		}
	}
	// Remember rotation based on center
	memset(rotprev, 0, sizeof(double)*360);
	memset(rot_count, 0, sizeof(int)*360);
	for (int y=0; y<y_res; y++) {
		for (int x=0; x<x_res; x++) {
			double y2 = y-(y_res/2);
			double x2 = x-(x_res/2);
			double r = sqrt((double)y2*y2 + x2*x2);
			int theta = int(atan2((double)y2, (double)x2)*180/3.14159265);
			if (theta < 0) theta+=360;
			if ((y >= 0) && (y < img->height) &&
				(x >= 0) && (x < img->width))
			{
				rotprev[theta] += (unsigned char)cvGet2D(img, y, x).val[0];
				rot_count[theta] ++;
			}
		}
	}
	for (int i=0; i<360; i++) {
		rotprev[i] /= rot_count[i];
	}
	return conf1 + best_rot_factor;
}

void TrackerPsaRot::Compensate(double *x, double *y)
{
	double xx = *x - (x_res/2);
	double yy = *y - (y_res/2);
	double kosini = cos(rotd*3.1415926535/180);
	double sini = sin(rotd*3.1415926535/180);
	*x = ((kosini * xx) - (sini * yy)) + xd + (x_res/2);
	*y = ((sini * xx) + (kosini * yy)) + yd + (y_res/2);
}

} // namespace alvar

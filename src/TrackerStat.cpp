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

#include "TrackerStat.h"

using namespace std;

namespace alvar {
using namespace std;

TrackerStat::TrackerStat(int binsize) : f(100,90) {
	hist.AddDimension(binsize); // x
	hist.AddDimension(binsize); // y
}

void TrackerStat::Reset() {
	f.Reset();
}

double TrackerStat::Track(IplImage *img) 
{
	if (img == NULL) return -1;
	f.Track(img);
	hist.Clear();
	for (int p=0; p<f.prev_feature_count ; p++) {
		for (int c=0; c<f.feature_count; c++) {
			if (f.prev_ids[p] != f.ids[c]) continue;
			float x = f.features[c].x - f.prev_features[p].x;
			float y = f.features[c].y - f.prev_features[p].y;
			hist.Inc(x, y);
		}
	}
	xd = 0; yd = 0;
	return hist.GetMax(&xd, &yd);
}

void TrackerStat::Compensate(double *x, double *y) { 
	*x += xd; *y += yd; 
}

TrackerStatRot::TrackerStatRot(int binsize /*=8*/, int binsize_rot/*=3*/) : TrackerStat(binsize) {
	hist_rot.AddDimension(binsize_rot);
}

double TrackerStatRot::Track(IplImage *img)
{
	if (img == NULL) return -1;
	f.Track(img);
	// Translation
	hist.Clear();
	for (int p=0; p<f.prev_feature_count ; p++) {
		for (int c=0; c<f.feature_count; c++) {
			if (f.prev_ids[p] != f.ids[c]) continue;
			float x = f.features[c].x - f.prev_features[p].x;
			float y = f.features[c].y - f.prev_features[p].y;
			hist.Inc(x, y);
		}
	}
	xd = 0; yd = 0;
	double ret = hist.GetMax(&xd, &yd);
	// Rotation
	x_res = img->width;
	y_res = img->height;
	hist_rot.Clear();
	for (int p=0; p<f.prev_feature_count ; p++) {
		for (int c=0; c<f.feature_count; c++) {
			if (f.prev_ids[p] != f.ids[c]) continue;
			double x_pred = f.prev_features[p].x + xd;
			double y_pred = f.prev_features[p].y + yd;
			double x_curr = f.features[c].x;
			double y_curr = f.features[c].y;
			double x = x_curr - x_pred;
			double y = y_curr - y_pred;
			double theta_pred = atan2((double)y_pred-(y_res/2), (double)x_pred-(x_res/2))*180.0/3.1415926535;
			double theta_curr = atan2((double)y_curr-(y_res/2), (double)x_curr-(x_res/2))*180.0/3.1415926535;
			hist_rot.Inc(theta_curr-theta_pred);
		}
	}
	rotd=0;
	hist_rot.GetMax(&rotd);
	return ret;
}

void TrackerStatRot::Compensate(double *x, double *y)
{
	double xx = *x - (x_res/2);
	double yy = *y - (y_res/2);
	double kosini = cos(rotd*3.1415926535/180);
	double sini = sin(rotd*3.1415926535/180);
	*x = ((kosini * xx) - (sini * yy)) + xd + (x_res/2);
	*y = ((sini * xx) + (kosini * yy)) + yd + (y_res/2);
}

} // namespace alvar

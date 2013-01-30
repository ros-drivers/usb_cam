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

#ifndef TRACKERSTAT_H
#define TRACKERSTAT_H

#include "Tracker.h"
#include "TrackerFeatures.h"
#include "Util.h"

/**
 * \file TrackerStat.h
 *
 * \brief This file implements a statistical tracker.
 */

namespace alvar {

/**
 * \brief \e TrackerStat deduces the optical flow based on tracked features using Seppo Valli's statistical tracking.
 */
class ALVAR_EXPORT TrackerStat : public Tracker {
protected:
	TrackerFeatures f;
	HistogramSubpixel hist;
public:
	/** \brief \e Track result x-translation in pixels */
	double xd;
	/** \brief \e Track result y-translation in pixels */
	double yd;
	/** \brief Constructor */
	TrackerStat(int binsize=8);
	/** \brief Reset */
	void Reset();
	/**
	 * \brief Translation tracker (the simplest possible)
	 */
	double Track(IplImage *img);
	virtual void Compensate(double *x, double *y);
};

/**
 * \brief TrackerStatRot implements a slightly extended version of TrackerStat which can also detect sideways rotation.
 */
class ALVAR_EXPORT TrackerStatRot : public TrackerStat {
	int x_res, y_res;
	HistogramSubpixel hist_rot;
public:
	/** \brief \e Track result rotation in degrees */
	double rotd;
	/** \brief Constructor */
	TrackerStatRot(int binsize=8, int binsize_rot=3);
	/**
	 * \brief Translation + rotation tracker
	 */
	double Track(IplImage *img);
	virtual void Compensate(double *x, double *y);
};

} // namespace alvar

#endif



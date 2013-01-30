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

#ifndef TRACKERPSA_H
#define TRACKERPSA_H

#include "Tracker.h"

/**
 * \file TrackerPsa.h
 *
 * \brief This file implements a PSA tracker.
 */

namespace alvar {

/**
 * \brief \e TrackerPsa implements a very simple PSA tracker 
 *
 * (See Drab, Stephan A. & Artner, Nicole M. "Motion Detection as Interaction Technique for Games & Applications on Mobile Devices" PERMID 2005) 
 */
class ALVAR_EXPORT TrackerPsa : public Tracker {
protected:
	int max_shift;
	int x_res, y_res;
	long *hor, *horprev;
	long *ver, *verprev;
	long framecount;

public:
	/** \brief \e Track result x-translation in pixels */
	double xd;
	/** \brief \e Track result y-translation in pixels */
	double yd;
	/** \brief Constructor */
	TrackerPsa(int _max_shift = 50);
	/** \brief Destructor */
	~TrackerPsa();
	/** \brief Track using PSA */
	double Track(IplImage *img);

	virtual void Compensate(double *x, double *y);
};

/**
 * \brief \e TrackerPsaRot implements a slightly extended version of a \e TrackerPsa which can also detect sideways rotation
 */
class ALVAR_EXPORT TrackerPsaRot : public TrackerPsa {
protected:
	double *rot, *rotprev;
	int *rot_count;

public:
	/** \brief \e Track result rotation in degrees */
	double rotd;
	/** \brief Constructor */
	TrackerPsaRot(int _max_shift = 50);
	/** \brief Destructor */
	~TrackerPsaRot();
	/** \brief Track using PSA with rotation*/
	double Track(IplImage *img);

	virtual void Compensate(double *x, double *y);
};

} // namespace alvar

#endif



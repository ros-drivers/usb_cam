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

#ifndef TRACKER_H
#define TRACKER_H

/**
 * \file Tracker.h
 *
 * \brief This file implements a tracking interface.
 */

#include <cxcore.h>
#include "Alvar.h"

namespace alvar {

/**
 * \brief Pure virtual base class for tracking optical flow
 *
 * The idea is to make own versions of \e Track method which updates the class member variables accordingly
 */
class ALVAR_EXPORT Tracker {
public:
	Tracker() {}
	/** \brief Pure virtual function for making the next track step. This analyses the image and  updates class member variables accordingly */
	virtual double Track(IplImage *img) = 0;

	virtual void Compensate(double *x, double *y) {}
};

} // namespace alvar

#endif

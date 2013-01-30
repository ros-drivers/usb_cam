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

#ifndef _LINE_H
#define _LINE_H

/**
 * \file Line.h
 *
 * \brief This file implements a parametrized line.
 */

#include "Alvar.h"
#include "Util.h"

namespace alvar {

/**
 * \brief Struct representing a line. The line is parametrized by its center and direction vector.
 */
struct ALVAR_EXPORT Line
{
	/**
	* \brief Constructor.
	* \param params params[0] and params[1] are the x and y components of the direction vector, params[2] and params[3] are the x and y coordinates of the line center.
	*/
	Line(float params[4]);
	Line()
	{}

	/**
	 * \brief Line center.
	*/
	PointDouble c; // center
	/**
	 * \brief Direction vector.
	*/
	PointDouble s; // direction vector
};

/**
 * \brief Fit lines to vector of points. 
 * \param lines		Resulting set of lines.
 * \param corners	Index list of line breaks.
 * \param edge		Vector of points (pixels) where the line is fitted.
 * \param grey		In the future, we may want to fit lines directly to grayscale image instead of thresholded edge.
 */
int ALVAR_EXPORT FitLines(std::vector<Line> &lines,
			 const std::vector<int>& corners,
			 const std::vector<PointInt >& edge,
					IplImage *grey=0); 

/**
 * \brief Calculates an intersection point of two lines.
 * \param l1	First line.
 * \param l2	Second line.
 * \return		Intersection point.
 */
PointDouble ALVAR_EXPORT Intersection(const Line& l1, const Line& l2);

} // namespace alvar 

#endif










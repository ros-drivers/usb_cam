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

#ifndef MULTIMARKERFILTERED_H
#define MULTIMARKERFILTERED_H

/**
 * \file MultiMarkerFiltered.h
 *
 * \brief This file implements an approximation algorithm to create a
 * multi-marker field configuration.
 */

#include "MultiMarker.h"

namespace alvar {

/**
 * \brief Multi marker that is constructed by first calculating the marker
 * poses directly relative to base marker and then filtering the results
 * using e.g. median filter.
 *
 * This can be used to initialize the marker field for MultiMarkerBundle class.
 */
class ALVAR_EXPORT MultiMarkerFiltered : public MultiMarker
{
protected:
	static const int filter_buffer_max=15;
	FilterMedian *pointcloud_filtered;

	double _Update(MarkerIterator &begin, MarkerIterator &end, 
                 Camera* cam, Pose& pose, IplImage* image);

public:
	/** \brief Constructor.
		\param indices Vector of marker codes that are included into multi marker. The first element defines origin.
	*/
	MultiMarkerFiltered(std::vector<int>& indices);

	/** \brief Destructor. */
	~MultiMarkerFiltered();

	/** \brief Updates the 3D point cloud by averaging the calculated results.
		\param marker_id The id of the marker whose corner positions are updated.
		\param edge_length The edge length of the marker.
		\param pose Current camera pose that is used to estimate the marker position.
	*/
	void PointCloudAverage(int marker_id, double edge_length, Pose &pose);

	/** \brief Updates the marker field and camera pose.
		\param markers Markers seen by the camera.
		\param camera Camera object used to detect markers.
		\param pose Current camera pose. This is also updated.
		\param image If != 0 some visualization will be drawn.
	*/
	template <class M>
	double Update(const std::vector<M>* markers, Camera* cam, Pose& pose, IplImage* image = 0)
	{
		if(markers->size() < 1) return false;
		MarkerIteratorImpl<M> begin(markers->begin());
		MarkerIteratorImpl<M> end(markers->end());
    return _Update(begin, end, 
                   cam, pose, image);
	}

	/**
	 * \brief Reset the measurements
	 */
	void MeasurementsReset() {
		pointcloud_filtered->reset();
	}
};

} // namespace alvar

#endif

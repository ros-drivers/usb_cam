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

#ifndef MULTIMARKERINITIALIZER_H
#define MULTIMARKERINITIALIZER_H

/**
 * \file MultiMarkerInitializer.h
 *
 * \brief This file implements a initialization algorithm to create a
 * multi-marker field configuration.
 */

#include "MultiMarker.h"

namespace alvar {

/**
 * \brief Initializes multi marker by estimating their relative positions
 * from one or more images.
 *
 * To use, detect markers from images using MarkerDetector and call
 * addMeasurement for each image that has at least two markers.
 * Finally, call initialize to compute the relative positions of markers.
 *
 * After the multi marker has been initialized, the point cloud can
 * be copied into another MultiMarker implementation by PointCloudCopy.
 */
class ALVAR_EXPORT MultiMarkerInitializer : public MultiMarker
{
public:
    /**
     * \brief MarkerMeasurement that holds the maker id.
     */
	class MarkerMeasurement : public Marker {
		long _id;
	public:
		MarkerMeasurement() : globalPose(false) {}
		bool globalPose;
		unsigned long GetId() const { return _id; }
		void SetId(unsigned long _id) { this->_id = _id; }
    };

protected:
	std::vector<bool> marker_detected;
	std::vector<std::vector<MarkerMeasurement> > measurements;
	typedef std::vector<std::vector<MarkerMeasurement> >::iterator MeasurementIterator;
	FilterMedian *pointcloud_filtered;
	int filter_buffer_min;

	bool updateMarkerPoses(std::vector<MarkerMeasurement> &markers, const Pose &pose);
	void MeasurementsAdd(MarkerIterator &begin, MarkerIterator &end);

public:
	MultiMarkerInitializer(std::vector<int>& indices, int filter_buffer_min = 4, int filter_buffer_max = 15);
	~MultiMarkerInitializer();

	/**
	 * Adds a new measurement for marker field initialization.
	 * Each measurement should contain at least two markers.
	 * It does not matter which markers are visible, especially
	 * the zero marker does not have to be visible in every measurement.
	 * It suffices that there exists a 'path' from the zero marker
	 * to every other marker in the marker field.
	 *
	 * For example:
	 *  - first measurement contains marker A and B.
	 *  - second measurement containt markers ZERO and A.
	 *
	 * When Initialize is called, the system can first deduce the pose of A 
	 * and then the pose of B.
	 */
	template <class M>
	void MeasurementsAdd(const std::vector<M> *markers) {
	    MarkerIteratorImpl<M> begin(markers->begin());
	    MarkerIteratorImpl<M> end(markers->end());
        MeasurementsAdd(begin, end);
	}

	/**
	 * 
	 */
	void MeasurementsReset();

	/**
	 * Tries to deduce marker poses from measurements.
	 *
	 * Returns the number of initialized markers.
	 */
	int Initialize(Camera* cam);

	int getMeasurementCount() { return measurements.size(); }

	const std::vector<MarkerMeasurement>& getMeasurementMarkers(int measurement) { 
		return measurements[measurement];
	}

	double getMeasurementPose(int measurement, Camera *cam, Pose &pose) {
		MarkerIteratorImpl<MarkerMeasurement> m_begin(measurements[measurement].begin());
		MarkerIteratorImpl<MarkerMeasurement> m_end(measurements[measurement].end());
		return _GetPose(m_begin, m_end, cam, pose, NULL);
	}
};

} // namespace alvar

#endif

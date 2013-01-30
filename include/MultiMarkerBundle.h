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

#ifndef MULTIMARKERBUNDLE_H
#define MULTIMARKERBUNDLE_H

/**
 * \file MultiMarkerBundle.h
 *
 * \brief This file implements an algorithm to create a multi-marker
 * field configuration.
 */

#include "MultiMarker.h"
#include "Optimization.h"

namespace alvar {

/**
 * \brief Multi marker that uses bundle adjustment to refine the 3D positions of the markers (point cloud).
 *
 * This can be initialized by using e.g. MultiMarkerAverage class.
 */
class ALVAR_EXPORT MultiMarkerBundle : public MultiMarker
{
protected:
	int optimization_keyframes;
	int optimization_markers;
	double optimization_error;
	bool optimizing;
	std::vector<Pose> camera_poses; // Estimated camera pose for every frame
	std::map<int, PointDouble> measurements; //
	int measurements_index(int frame, int marker_id, int marker_corner) {
		// hop return (int) (frame*marker_indices.size()*4)+(marker_id*4)+marker_corner;
		return (int) (frame*marker_indices.size()*4)+(get_id_index(marker_id)*4)+marker_corner;
	}

	void _MeasurementsAdd(MarkerIterator &begin, MarkerIterator &end, const Pose& camera_pose);

public:

	/** \brief Constructor.
		\param indices Vector of marker codes that are included into multi marker. The first element defines origin.
	*/
	MultiMarkerBundle(std::vector<int>& indices);

	~MultiMarkerBundle();

	/** \brief Resets the measurements and camera poses that are stored for bundle adjustment. If something goes from in 
			   optimization one will call this and acquire new measurements.
	*/
	void MeasurementsReset();

	double GetOptimizationError() { return optimization_error; }
	int GetOptimizationKeyframes() { return optimization_keyframes; }
	int GetOptimizationMarkers() { return optimization_markers; }
	bool GetOptimizing() { return optimizing; }

	/** \brief Adds new measurements that are used in bundle adjustment.
		\param markers Vector of markers detected by MarkerDetector.
		\param camera_pose Current camera pose.
	*/
	template <class M>
	void MeasurementsAdd(const std::vector<M> *markers, const Pose& camera_pose) {
	    MarkerIteratorImpl<M> begin(markers->begin());
	    MarkerIteratorImpl<M> end(markers->end());
        _MeasurementsAdd(begin, end,
                     camera_pose);
	}

	/** \brief Runs the bundle adjustment optimization.
		\param markers Vector of markers detected by MarkerDetector.
		\param camera_pose Current camera pose.
		\param max_iter Maximum number of iteration loops.
		\param method The method that is applied inside optimization. Try Optimization::LEVENBERGMARQUARDT or Optimization::GAUSSNEWTON or Optmization::TUKEY_LM
	*/																											//LEVENBERGMARQUARDT
	bool Optimize(Camera *_cam, double stop, int max_iter, Optimization::OptimizeMethod method = Optimization::TUKEY_LM); //TUKEY_LM
};

} // namespace alvar

#endif

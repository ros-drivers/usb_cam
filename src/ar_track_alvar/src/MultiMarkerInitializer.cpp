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

#include "MultiMarkerInitializer.h"

using namespace std;

namespace alvar {
using namespace std;

MultiMarkerInitializer::MultiMarkerInitializer(std::vector<int>& indices, int _filter_buffer_min, int _filter_buffer_max)
	: MultiMarker(indices), filter_buffer_min(_filter_buffer_min) {

	marker_detected.resize(indices.size());
	pointcloud_filtered = new FilterMedian[indices.size()*4*3];
	for (size_t i=0; i<indices.size()*4*3; i++) {
		pointcloud_filtered[i].setWindowSize(_filter_buffer_max);
	}

	MeasurementsReset();
}

void MultiMarkerInitializer::MeasurementsAdd(MarkerIterator &begin, MarkerIterator &end) {
	// copy markers into measurements.
	vector<MarkerMeasurement> new_measurements;
	for (MarkerIterator &i = begin.reset(); i != end; ++i) {
		const Marker* marker = *i;
		int index = get_id_index(marker->GetId()); 
		if (index == -1) continue;
		MarkerMeasurement m;
		m.SetId(marker->GetId());
		m.SetMarkerSize(marker->GetMarkerEdgeLength(), marker->GetRes(), marker->GetMargin());
		m.pose = marker->pose;
		m.marker_corners_img = i->marker_corners_img;
		new_measurements.push_back(m);
		marker_detected[index] = true;
	}

	// If we have seen the 0 marker the first time we push it into point could.
	for (MarkerIterator &i = begin.reset(); i != end; ++i) {
		const Marker* marker = *i;
		int id = marker->GetId();
		int index = get_id_index(id);
		// Initialize marker pose
		//if (id == 0 && marker_status[index] == 0)
		if (index == 0 && marker_status[index] == 0)
		{
			Pose pose;
			CvPoint3D64f corners[4];
			PointCloudCorners3d(marker->GetMarkerEdgeLength(), pose, corners);
			for(size_t j = 0; j < 4; ++j) {
				int p_index = pointcloud_index(id, j);
				pointcloud[p_index] = corners[j];
			}
			marker_status[index] = 1;
		}
	}

	measurements.push_back(new_measurements);
}

void MultiMarkerInitializer::MeasurementsReset() {
	measurements.clear();
	PointCloudReset();
	fill(marker_status.begin(), marker_status.end(), 0);
	fill(marker_detected.begin(), marker_detected.end(), false);

	for (size_t i=0; i<marker_indices.size()*4*3; i++) {
		pointcloud_filtered[i].reset();
	}
}

int MultiMarkerInitializer::Initialize(Camera* cam) {
	for (bool found_new = true; found_new; ) {
		found_new = false;
		// Iterate through all measurements, try to compute Pose for each.
		for (MeasurementIterator mi = measurements.begin(); mi != measurements.end(); ++mi) {
			vector<MarkerMeasurement> &markers = *mi;
			Pose pose;
			MarkerIteratorImpl<MarkerMeasurement> m_begin(markers.begin());
			MarkerIteratorImpl<MarkerMeasurement> m_end(markers.end());
			double err = _GetPose(m_begin, m_end, cam, pose, NULL);
			if (err >= 0) {
				// If pose is found, estimate marker poses for those that are still unkown.
				found_new = updateMarkerPoses(markers, pose);
			}
		}
	}

	// Check if every marker has been seen
	int n_detected = 0;
	for (unsigned int i = 0; i < marker_indices.size(); ++i) {
		cout << i << " " << marker_detected[i] << " " << marker_status[i] << "\n";
		if (marker_detected[i] && marker_status[i] != 0) ++n_detected;
	}
	return n_detected;
}

bool MultiMarkerInitializer::updateMarkerPoses(vector<MarkerMeasurement> &markers, const Pose &pose) {
	bool found_new = false;
	for (vector<MarkerMeasurement>::iterator i = markers.begin(); i != markers.end(); ++i) {
		MarkerMeasurement &marker = *i;
		int id = marker.GetId();
		int index = get_id_index(id);
		if (index > 0 && !marker.globalPose) {
			// Compute absolute marker pose.
			double cam_posed[16];
			double mar_posed[16];
			CvMat cam_mat = cvMat(4, 4, CV_64F, cam_posed);
			CvMat mar_mat = cvMat(4, 4, CV_64F, mar_posed);
			pose.GetMatrix(&cam_mat);
			marker.pose.GetMatrix(&mar_mat);
			cvInvert(&cam_mat, &cam_mat);
			cvMatMul(&cam_mat, &mar_mat, &mar_mat);
			marker.pose.SetMatrix(&mar_mat);
			// Put marker into point cloud
			CvPoint3D64f corners[4];
			PointCloudCorners3d(marker.GetMarkerEdgeLength(), marker.pose, corners);
			for(size_t j = 0; j < 4; ++j) {
				CvPoint3D64f p;
				int p_index = pointcloud_index(id, j);
				p.x = pointcloud_filtered[3*p_index+0].next(corners[j].x);
				p.y = pointcloud_filtered[3*p_index+1].next(corners[j].y);
				p.z = pointcloud_filtered[3*p_index+2].next(corners[j].z);
				if (pointcloud_filtered[3*p_index+0].getCurrentSize() >= filter_buffer_min) {
					pointcloud[p_index] = p;
					marker_status[index] = 1;
				}
			}
			marker.globalPose = 1;
			found_new = true;
		}
	}
	return found_new;
}

} // namespace alvar

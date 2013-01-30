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

#include "MultiMarkerFiltered.h"

using namespace std;

namespace alvar {
using namespace std;

MultiMarkerFiltered::MultiMarkerFiltered(std::vector<int>& indices)
	: MultiMarker(indices)
{
	pointcloud_filtered = new FilterMedian[indices.size()*4*3];
	for (size_t i=0; i<indices.size()*4*3; i++) {
		pointcloud_filtered[i].setWindowSize(filter_buffer_max);
	}
}

MultiMarkerFiltered::~MultiMarkerFiltered()
{
	delete [] pointcloud_filtered;
}

void MultiMarkerFiltered::PointCloudAverage(int marker_id, double edge_length, Pose &pose) {
	if (marker_id == 0) {
		if (marker_status[get_id_index(marker_id)] == 0) PointCloudAdd(marker_id, edge_length, pose);
	} else {
		CvPoint3D64f corners[4];
		PointCloudCorners3d(edge_length, pose, corners);
		for(size_t j = 0; j < 4; ++j) {
			int id_index = get_id_index(marker_id);
			if (id_index < 0) continue;
			int index = id_index*4*3 + j*3;
			CvPoint3D64f p;
			p.x = pointcloud_filtered[index+0].next(corners[j].x);
			p.y = pointcloud_filtered[index+1].next(corners[j].y);
			p.z = pointcloud_filtered[index+2].next(corners[j].z);
			pointcloud[pointcloud_index(marker_id, j)] = p;
			// The marker isn't used for pose calculation until we are quite sure about it ???
			if (pointcloud_filtered[index+0].getCurrentSize() >= filter_buffer_max) {
				marker_status[get_id_index(marker_id)]=1;
			}
		}
	}
}

double MultiMarkerFiltered::_Update(MarkerIterator &begin, MarkerIterator &end, 
                                    Camera* cam, Pose& pose, IplImage* image)
{
	if (_GetPose(begin, end, cam, pose, NULL) == -1) return -1;

	// For every detected marker
  for (MarkerIterator &i = begin.reset(); i != end; ++i)
	{
		const Marker* marker = *i;
		int id = marker->GetId();
		int index = get_id_index(id);
		if (index < 0) continue;

		// Initialize marker pose
		if (marker_status[index] == 0)
		{
			double cam_posed[16];
			double mar_posed[16];

			CvMat cam_mat = cvMat(4, 4, CV_64F, cam_posed);
			CvMat mar_mat = cvMat(4, 4, CV_64F, mar_posed);

			pose.GetMatrix(&cam_mat);
			marker->pose.GetMatrix(&mar_mat);

			cvInvert(&cam_mat, &cam_mat);
			cvMatMul(&cam_mat, &mar_mat, &mar_mat);

			Pose p;
			p.SetMatrix(&mar_mat);
			PointCloudAverage(id, marker->GetMarkerEdgeLength(), p);
		}
	}

	// When the pointcloud is updated we will get the new "better" pose...
	return _GetPose(begin, end, cam, pose, image);
}

} // namespace alvar

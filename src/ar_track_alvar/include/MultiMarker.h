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

#ifndef MULTIMARKER_H
#define MULTIMARKER_H

/**
 * \file MultiMarker.h
 *
 * \brief This file implements a multi-marker.
 */

#include "Alvar.h"
#include "Rotation.h"
#include "MarkerDetector.h"
#include "Camera.h"
#include "Filter.h"
#include "FileFormat.h"
#include <tf/LinearMath/Vector3.h>

namespace alvar {

/**
 * \brief Base class for using MultiMarker.
 */
class ALVAR_EXPORT MultiMarker {


private:

	bool SaveXML(const char* fname);
	bool SaveText(const char* fname);
	bool LoadText(const char* fname);
	bool LoadXML(const char* fname);

public:
    // The marker information is stored in all three tables using 
	// the indices-order given in constructor. 
	// One idea is that the same 'pointcloud' could contain feature 
	// points after marker-corner-points. This way they would be
	// optimized simultaneously with marker corners...
	std::map<int, CvPoint3D64f> pointcloud;
	std::vector<int> marker_indices; // The marker id's to be used in marker field (first being the base)
	std::vector<int> marker_status;  // 0: not in point cloud, 1: in point cloud, 2: used in GetPose()
    std::vector< std::vector<tf::Vector3> > rel_corners; //The coords of the master marker relative to each child marker in marker_indices

	int pointcloud_index(int marker_id, int marker_corner, bool add_if_missing=false);
	int get_id_index(int id, bool add_if_missing=false);

	double _GetPose(MarkerIterator &begin, MarkerIterator &end, Camera* cam, Pose& pose, IplImage* image);

	int _SetTrackMarkers(MarkerDetectorImpl &marker_detector, Camera* cam, Pose& pose, IplImage *image);
	int master_id;  //The id of the first marker specified in the XML file 


	/** \brief Resets the multi marker. */
	virtual void Reset();

	/** \brief Saves multi marker configuration to a text file.
	 * \param fname file name
	 * \param format FILE_FORMAT_TEXT (default) or FILE_FORMAT_XML (see doc/MultiMarker.xsd).
	 */
	bool Save(const char* fname, FILE_FORMAT format = FILE_FORMAT_DEFAULT);

	/** \brief Loads multi marker configuration from a text file.
	 * \param fname file name
	 * \param format FILE_FORMAT_TEXT (default) or FILE_FORMAT_XML (see doc/MultiMarker.xsd).
	 */
	bool Load(const char* fname, FILE_FORMAT format = FILE_FORMAT_DEFAULT);

	/** \brief Constructor.
		\param indices Vector of marker codes that are included into multi marker. The first element defines origin.
	*/
	MultiMarker(std::vector<int>& indices);

	/** \brief Default constructor */
	MultiMarker() {}

	/** \brief Calculates the pose of the camera from multi marker. Method uses the true 3D coordinates of 
		 markers to get the initial pose and then optimizes it by minimizing the reprojection error.

		\param markers Vector of markers seen by camera.
		\param cam Camera object containing internal calibration etc.
		\param pose The resulting pose is stored here.
		\param image If != 0 some visualizations are drawn.
	*/
	template <class M>
	double GetPose(const std::vector<M>* markers, Camera* cam, Pose& pose, IplImage* image = 0)
	{
		MarkerIteratorImpl<M> begin(markers->begin());
		MarkerIteratorImpl<M> end(markers->end());
		return _GetPose(begin, end,
						cam, pose, image);
	}

	/** \brief Calls GetPose to obtain camera pose.
	*/
	template <class M>
	double Update(const std::vector<M>* markers, Camera* cam, Pose& pose, IplImage* image = 0)
	{
		if(markers->size() < 1) return -1.0;

		return GetPose(markers, cam, pose, image);
	}

	/** \brief Reserts the 3D point cloud of the markers.
	*/
	void PointCloudReset();

	/** \brief Calculates 3D coordinates of marker corners relative to given pose (camera).

		\param edge_length The edge length of the marker.
		\param pose Current pose of the camera.
		\param corners Resulted 3D corner points are stored here.
	*/
	void PointCloudCorners3d(double edge_length, Pose &pose, CvPoint3D64f corners[4]);

	/** \brief Adds marker corners to 3D point cloud of multi marker.
		\param marker_id Id of the marker to be added.
		\param edge_length Edge length of the marker.
		\param pose Current camera pose.
	*/
	void PointCloudAdd(int marker_id, double edge_length, Pose &pose);

	/** \brief Copies the 3D point cloud from other multi marker object.
	*/
	void PointCloudCopy(const MultiMarker *m);

	/** \brief Returns true if the are not points in the 3D opint cloud.
	 */
	bool PointCloudIsEmpty() {
		return pointcloud.empty();
	}

	/** \brief Return the number of markers added using PointCloudAdd */
	size_t Size() {
		return marker_indices.size();
	}

	/** \brief Returns 3D co-ordinates of one corner point of a marker.
	 * \param marker_id ID of the marker which corner point is returned.
	 * \param point Number of the corner point to return, from 0 to 3.
	 * \param x x co-ordinate is returned.
	 * \param y y co-ordinate is returned.
	 * \param z z co-ordinate is returned.
	 */
	void PointCloudGet(int marker_id, int point,
	double &x, double &y, double &z);

	/** \brief Returns true if the marker is in the point cloud.
	 * \param marker_id ID of the marker.
	 */
	bool IsValidMarker(int marker_id);

	std::vector<int> getIndices(){
		return marker_indices;
	}

	int getMasterId(){
		return master_id;
	}

	/** \brief Set new markers to be tracked for \e MarkerDetector
	 * Sometimes the \e MultiMarker implementation knows more about marker
	 * locations compared to \e MarkerDetector . Then this method can be
	 * used to reset the internal state of \e MarkerDetector for tracking
	 * also these markers.
	*/
	template <class M>
	int SetTrackMarkers(MarkerDetector<M> &marker_detector, Camera* cam, Pose& pose, IplImage *image=0) {
    return _SetTrackMarkers(marker_detector, cam, pose, image);
	}
};

} // namespace alvar

#endif

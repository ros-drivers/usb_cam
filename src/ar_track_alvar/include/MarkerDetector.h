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

#ifndef MARKER_DETECTOR_H
#define MARKER_DETECTOR_H

/**
 * \file MarkerDetector.h
 *
 * \brief This file implements a generic marker detector.
 */

#include "Alvar.h"
#include "Util.h"
#include "ConnectedComponents.h"
#include "Draw.h"
#include "Camera.h"
#include "Marker.h"
#include "Rotation.h"
#include "Line.h"
#include <algorithm>
using std::rotate;
#include <list>
#include <vector>
#include <map>
#include <cassert>

namespace alvar {

/**
 * \brief Templateless version of MarkerDetector. Please use MarkerDetector instead.
 */
class ALVAR_EXPORT MarkerDetectorImpl {
protected:
	virtual Marker* new_M(double _edge_length = 0, int _res = 0, double _margin = 0) = 0;
	virtual void _markers_clear() = 0;
	virtual void _markers_push_back(Marker *mn) = 0;
	virtual size_t _markers_size() = 0;
	virtual void _track_markers_clear() = 0;
	virtual void _track_markers_push_back(Marker *mn) = 0;
	virtual size_t _track_markers_size() = 0;
	virtual Marker* _track_markers_at(size_t i) = 0;
	virtual void _swap_marker_tables() = 0;

	Labeling* labeling;

	std::map<unsigned long, double> map_edge_length;
	double edge_length;
	int res;
	double margin;
	bool detect_pose_grayscale;

	MarkerDetectorImpl();
	virtual ~MarkerDetectorImpl();

public:
	/** \brief Clear the markers that are tracked */
	void TrackMarkersReset();

	/** \brief Add markers to be tracked 
	 * Sometimes application or e.g. the \e MultiMarker implementation knows 
	 * more about marker locations. Then this method can be used after \e Detect
	 * to indicate where additional trackable markers could be found. The
	 * \e DetectAdditional is called for tracking these.
	 */
	void TrackMarkerAdd(int id, PointDouble corners[4]);

	/** Set the default marker size to be used for all markers unless 
	* \param _edge_length Length of the marker's edge in whatever units you are using (e.g. cm)
	* \param _res The marker content resolution in pixels. By default we use 5x5 markers. If you use 0 with \e MarkerData, the marker resolution is detected automatically.
	* \param _margin The marker margin resolution in pixels (The actual captured marker image has pixel resolution of _margin+_res+_margin)
    *
    * \note The default marker content resolution (_res) of 5 can only detect marker ids from 0 to 255. For larger marker ids, you need to increase the marker content resolution accordingly.
	*/
	void SetMarkerSize(double _edge_length = 1, int _res = 5, double _margin = 2);

	/** Set marker size for specified marker id. This needs to be called after setting the default marker size.
	* \param id The specified marker id
	* \param _edge_length Length of the marker's edge in whatever units you are using (e.g. cm)
	*/
	void SetMarkerSizeForId(unsigned long id, double _edge_length = 1);

	/** Set marker size for specified marker id. This needs to be called after setting the default marker size.
	* \param _detect_pose_grayscale Do we detect marker pose using grayscale optimization?
	*/
	void SetOptions(bool _detect_pose_grayscale=false);

	/**
	 * \brief \e Detect \e Marker 's from \e image 
	 *
	 * The coordinates are little tricky. Here is a short summary.
	 * 
	 * - Image (top-left origin).
	 * - The marker corners in the image are searched in sub-pixel accuracy in 
	 *   counter-clockwise order starting from "lower-left" corner.
	 * - The corresponding marker corners and marker points are in marker coordinates
	 *   (x is to east, y is to north, and z is up from the marker)
	 * - The marker points are read from inside the margins starting from top-left
	 *   and reading the bits first left-to-right one line at a time.
	 */
	int Detect(IplImage *image,
			   Camera *cam,
			   bool track=false,
			   bool visualize=false,
			   double max_new_marker_error=0.08,
			   double max_track_error=0.2,
			   LabelingMethod labeling_method=CVSEQ,
			   bool update_pose=true);

	int DetectAdditional(IplImage *image, Camera *cam, bool visualize=false, double max_track_error=0.2);
};

/**
 * \brief \e MarkerDetector for detecting markers of type \e M
 * \param M Class that extends \e Marker
 */
template<class M>
class ALVAR_EXPORT MarkerDetector : public MarkerDetectorImpl
{
protected:
  Marker* new_M(double _edge_length = 0, int _res = 0, double _margin = 0) {
    return new M(_edge_length, _res, _margin);
  }

  void _markers_clear() { markers->clear(); }
  void _markers_push_back(Marker *mn) { markers->push_back(*((M*)mn)); }
  size_t _markers_size() { return markers->size(); }
  void _track_markers_clear() { track_markers->clear(); }
  void _track_markers_push_back(Marker *mn) { track_markers->push_back(*((M*)mn)); }
  size_t _track_markers_size() { return track_markers->size(); }
  Marker* _track_markers_at(size_t i) { return &track_markers->at(i); }

  void _swap_marker_tables() {
		std::vector<M> *tmp_markers = markers;
		markers = track_markers;
		track_markers = tmp_markers;
  }

public:

	std::vector<M> *markers;
	std::vector<M> *track_markers;

	/** Constructor */
  MarkerDetector() : MarkerDetectorImpl() {
    markers = new std::vector<M>;
    track_markers = new std::vector<M>;
	}

	/** Destructor */
	~MarkerDetector() {
    delete markers;
    delete track_markers;
	}
};

} // namespace alvar

#endif

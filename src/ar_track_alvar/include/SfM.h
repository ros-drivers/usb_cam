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

#ifndef SFM_H
#define SFM_H

/**
 * \file SfM.h
 *
 * \brief This file implements structure from motion.
 */

#include "EC.h"

namespace alvar {

/** \brief Simple structure from motion implementation using \e CameraEC , \e MarkerDetectorEC and \e TrackerFeaturesEC 
 *
 *  See \e SamplePointcloud for usage example.
 */
class ALVAR_EXPORT SimpleSfM {
public:
	/** \brief Extended version of \e ExternalContainer structure used internally in \e SimpleSfM */
	class Feature : public ExternalContainer {
	public:
		bool has_stored_pose;
		Pose pose1;
		CvPoint2D32f p2d1;
		CvPoint3D32f p3d_sh;
		CvPoint2D32f projected_p2d_sh;
		int tri_cntr;        // This is needed only by UpdateTriangulateOnly
		int estimation_type;

		Feature() : ExternalContainer() {
			has_stored_pose = false;
			p3d_sh.x = 0.f; p3d_sh.y = 0.f; p3d_sh.z = 0.f;
			tri_cntr = 0;
			estimation_type = 0;
		}
		Feature(const Feature &c) : ExternalContainer(c) {
			has_stored_pose = c.has_stored_pose;
			pose1 = c.pose1;
			p2d1 = c.p2d1;
			p3d_sh = c.p3d_sh;
			projected_p2d_sh = c.projected_p2d_sh;
			tri_cntr = c.tri_cntr;
			estimation_type = c.estimation_type;
		}
	};
	/** \brief The map of all tracked features */
	std::map<int, Feature> container;
	std::map<int, Feature> container_triangulated;

protected:
	std::map<int, Feature> container_reset_point;
	std::map<int, Feature> container_triangulated_reset_point;

	CameraEC cam;
	MarkerDetectorEC<MarkerData> marker_detector; 
	TrackerFeaturesEC tf;
	Pose pose;
	bool pose_ok;

	bool update_tri;
	std::string multi_marker_file;
	bool markers_found;
	double scale;
	Pose pose_original;
	Pose pose_difference;

public:
	/** \brief Constructor */
	SimpleSfM() : tf(200, 200, 0.01, 20, 4, 6), pose_ok(false), update_tri(false), markers_found(false) {}
	/** \brief Reset the situation back to the point it was when \e SetResetPoint was called */
	void Reset(bool reset_also_triangulated = true);
	/** \brief Remember the current state and return here when the \e Reset is called */
	void SetResetPoint();
	/** \brief Clear all tracked features */
	void Clear();
	/** \brief Set the suitable scale to be used. This affects quite much how the tracking behaves (when features are triangulated etc.) */
	void SetScale(double s) { scale = s; }
	/** \brief Get the camera used internally. You need to use this to set correct camera calibration (see \e SamplePointcloud) */
	CameraEC *GetCamera();
	/** \brief Get the estimated pose */
	Pose *GetPose();
	/** \brief Add \e MultiMarker from file as a basis for tracking. It is good idea to call \e SetResetPoint after these. */
	bool AddMultiMarker(const char *fname, FILE_FORMAT format = FILE_FORMAT_XML);
	/** \brief Add \e MultiMarker to be a basis for tracking. It is good idea to call \e SetResetPoint after these. */
	bool AddMultiMarker(MultiMarkerEC *mm);
	/** \brief Add an marker to be a basis for tracking. It is good idea to call \e SetResetPoint after these. */
	void AddMarker(int marker_id, double edge_length, Pose &pose);
	/** \brief Update position assuming that camera is moving with 6-DOF */
	bool Update(IplImage *image, bool assume_plane=true, bool triangulate=true, float reproj_err_limit=5.f, float triangulate_angle=15.f);
	/** \brief Update camera 6-DOF position using triangulated points only (This is the old version of Update) */
	bool UpdateTriangulateOnly(IplImage *image);
	/** \brief Update position assuming that user is more standing still and viewing the environment. */
	bool UpdateRotationsOnly(IplImage *image);
	/** \brief Draw debug information about the tracked features and detected markers. */
	void Draw(IplImage *rgba);
};

}
#endif

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

#ifndef EC_H
#define EC_H

/**
 * \file EC.h
 *
 * \brief This file implements a collection of External Container (EC)
 * versions of many ALVAR classes.
 */

#include "TrackerFeatures.h"
#include "Camera.h"
#include "MarkerDetector.h"
#include "MultiMarker.h"

namespace alvar {

/** \brief Basic structure to be usable with EC methods.
 *
 * You can inherit your own classes/structures from this or make one with similar members.
 *
 * The idea is that the EC-versions store the data externally in a map of features that
 * are based on the structure \e ExternalContainer . The \e ExternalContainer has certain
 * predefined fields that are used/needed by different methods. Below is a summary of
 * some main methods for reading (r) and writing (w) in the external container.
 *
 *  <table border> 
 *  <tr> 
 *     <td></td> 
 *     <td><b>type_id</b></td> 
 *     <td><b>has_p2d</b></td> 
 *     <td><b>has_p3d</b></td> 
 *     <td><b>p2d</b></td> 
 *     <td><b>p3d</b></td> 
 *     <td><b>projected_p2d</b></td> 
 *  </tr> 
 *  <tr> 
 *     <td>TrackerFeaturesEC::Track</td> 
 *     <td>(r)w</td> 
 *     <td>rw</td>
 *     <td></td>
 *     <td>rw</td>
 *     <td></td>
 *     <td></td>
 *  </tr>
 *  <tr>
 *     <td>CameraEC::Undistort</td> 
 *     <td>(r)</td> 
 *     <td>rw</td>
 *     <td></td>
 *     <td>rw</td>
 *     <td></td>
 *     <td></td>
 *  </tr>
 *  <tr>
 *     <td>CameraEC::Distort</td> 
 *     <td>(r)</td> 
 *     <td>rw</td>
 *     <td></td>
 *     <td>rw</td>
 *     <td></td>
 *     <td></td>
 *  </tr>
 *  <tr>
 *     <td>CameraEC::CalcExteriorOrientation</td> 
 *     <td>(r)</td> 
 *     <td>r</td>
 *     <td>r</td>
 *     <td>r</td>
 *     <td>r</td>
 *     <td></td>
 *  </tr>
 *  <tr>
 *     <td>CameraEC::UpdatePose</td> 
 *     <td>(r)</td> 
 *     <td>r</td>
 *     <td>r</td>
 *     <td>r</td>
 *     <td>r</td>
 *     <td></td>
 *  </tr>
 *  <tr>
 *     <td>CameraEC::Reproject</td> 
 *     <td>(r)</td> 
 *     <td>(r)</td>
 *     <td>(r)</td>
 *     <td>r</td>
 *     <td>r</td>
 *     <td>w</td>
 *  </tr>
 *  <tr>
 *     <td>CameraEC::EraseUsingReprojectionError</td> 
 *     <td>(r)</td> 
 *     <td>(r)</td>
 *     <td>(r)</td>
 *     <td></td>
 *     <td></td>
 *     <td>r</td>
 *  </tr>
 *  <tr>
 *     <td>MarkerDetectorEC::Detect</td> 
 *     <td>rw</td> 
 *     <td>w</td>
 *     <td></td>
 *     <td>w</td>
 *     <td></td>
 *     <td></td>
 *  </tr>
 *  <tr>
 *     <td>MarkerDetectorEC::MarkerToEC</td> 
 *     <td>w</td> 
 *     <td></td>
 *     <td>w</td>
 *     <td></td>
 *     <td>w</td>
 *     <td></td>
 *  </tr>
 *  <tr>
 *     <td>MultiMarkerEC::MarkersToEC</td> 
 *     <td>w</td> 
 *     <td></td>
 *     <td>w</td>
 *     <td></td>
 *     <td>w</td>
 *     <td></td>
 *  </tr>
 *  </table> 
 */
class ExternalContainer {
public:
	int type_id;
	bool has_p2d;
	bool has_p3d;
	CvPoint2D32f p2d;
	CvPoint3D32f p3d;
	CvPoint2D32f projected_p2d; // This is only temporary -- user needs to take care that it has valid content
	ExternalContainer() : type_id(-1), has_p2d(false), has_p3d(false) {}
	ExternalContainer(const ExternalContainer &c) {
		type_id = c.type_id;
		has_p2d = c.has_p2d;
		has_p3d = c.has_p3d;
		p2d.x = c.p2d.x;
		p2d.y = c.p2d.y;
		p3d.x = c.p3d.x;
		p3d.y = c.p3d.y;
		p3d.z = c.p3d.z;
		projected_p2d.x = c.projected_p2d.x;
		projected_p2d.y = c.projected_p2d.y;
	}
};

/** \brief This is a default functor for testing which items in the container should be handled by each method 
 *
 *  By default this only tests that the only the \e type_id. If you specify \e type_id that is not -1
 *  then the functor tests that the item's \e type_id matches. Naturally you can make whatever tests
 *  in your own functors.
 */
template<typename T>
class DoHandleTest {
protected:
	int type_id;
	bool needs_has_p2d;
	bool needs_has_p3d;
public:
	DoHandleTest(int _type_id=-1, bool _needs_has_p2d=false, bool _needs_has_p3d=false) 
		: type_id(_type_id), needs_has_p2d(_needs_has_p2d), needs_has_p3d(_needs_has_p3d) {}
	virtual bool operator()(const T &f) const {
		if (needs_has_p2d && !f.has_p2d) return false;
		if (needs_has_p3d && !f.has_p3d) return false;
		// if (f.type_id == -1) return false; // Unnecessary?
		if ((type_id != -1) && (type_id != f.type_id)) return false;
		return true;
	}
};

/** \brief This is default functor for testing which items in the container should be erased
 *
 *  By default this can test for \e has_p2d and \e has_p3d . Furthermore, it can test for
 *  reprojection error when \e Pose , \e Camera and limit are given.
 */
template<typename T>
class DoEraseTest {
protected:
	int type_id;
	bool erase_without_p2d;
	bool erase_without_p3d;
	bool test_reprojection;
	float limit_sq;
public:
	DoEraseTest(bool _erase_without_p2d, bool _erase_without_p3d, int _type_id=-1) 
		: type_id(_type_id), erase_without_p2d(_erase_without_p2d), erase_without_p3d(_erase_without_p3d), 
		  test_reprojection(false), limit_sq(0.f) {}
	DoEraseTest(float _limit, bool _erase_without_p2d=false, bool _erase_without_p3d=false, int _type_id=-1) 
		: type_id(_type_id), erase_without_p2d(_erase_without_p2d), erase_without_p3d(_erase_without_p3d),
		  test_reprojection(true), limit_sq(_limit*_limit) {}
	virtual bool operator()(const T &f) const {
		if ((type_id != -1) && (type_id != f.type_id)) return false;
		if (erase_without_p2d && !f.has_p2d) return true;
		if (erase_without_p3d && !f.has_p3d) return true;
		if (test_reprojection) {
			if (!f.has_p2d) return false;
			if (!f.has_p3d) return false;
			// Note that the projected_p2d needs to have valid content at this point
			double dist_sq = (f.p2d.x-f.projected_p2d.x)*(f.p2d.x-f.projected_p2d.x);
			dist_sq       += (f.p2d.y-f.projected_p2d.y)*(f.p2d.y-f.projected_p2d.y);
			if (dist_sq > limit_sq) 
				return true;
		}
		return false;
	}
};

/** \brief Erasing items from container using \e DoEraseTest type functor */
template<typename T, typename F>
inline int EraseItemsEC(std::map<int,T> &container, F do_erase_test) {
	int count=0;
	typename std::map<int,T>::iterator iter = container.begin();
	while(iter != container.end()) {
		T &f = iter->second;
		if (do_erase_test(f)) {
			container.erase(iter++);
			count++;
		} else iter++;
	}
	return count;
}

/** \brief Version of \e TrackerFeatures using external container */
class TrackerFeaturesEC : public TrackerFeatures {
protected:
	bool purge;
public:
	/** \brief Constructor */
	TrackerFeaturesEC(int _max_features=100, int _min_features=90, double _quality_level=0.01, double _min_distance=10, int _pyr_levels=4, int win_size=6)
		: TrackerFeatures(_max_features, _min_features, _quality_level, _min_distance, _pyr_levels, win_size), purge(false) {}

	/** \brief Track features with matching type id. New features will have id's in the specified id range. */
	template<typename T>
	bool Track(IplImage *img, IplImage *mask, std::map<int,T> &container, int type_id=-1, int first_id=-1, int last_id=-1)
	{
		DoHandleTest<T> do_handle_test_default(type_id);
		if (type_id == -1) type_id=0;
		return Track(img, mask, container, do_handle_test_default, type_id, first_id, last_id);
	}

	/** \brief Track features matching the given functor. New features will have id's in the specified id range. */
	/*template<typename T, typename F>
	bool Track(IplImage *img, IplImage *mask, std::map<int,T> &container, F do_handle_test, int type_id=0, int first_id=0, int last_id=65535)
	{
		// Update features to match the ones in the given container
		typename std::map<int,T>::iterator iter = container.begin();
		typename std::map<int,T>::iterator iter_end = container.end();
		feature_count = 0;
		for (;iter != iter_end; iter++) {
			T &f = iter->second;
			if (!do_handle_test(f)) continue;
			if (f.has_p2d != true) continue;
			f.has_p2d = false; // This is updated again to true if tracking succeeds
			features[feature_count] = f.p2d;
			ids[feature_count] = iter->first;
			feature_count++;
			if (feature_count == max_features) break;
		}
		// Check that next_id is ok
		if (next_id < first_id)	next_id = first_id;
		if (next_id > last_id) return false; // TODO: Make some better solution for this
		// Purge if needed
		if (purge) {
			TrackerFeatures::Purge();
			purge=false;
		}
		// Track as usual (this will swap above features to prev_features)
		TrackHid(img, mask);
		// Update the container to have the updated features
		for (int i=0; i<feature_count; i++) {
			int id = ids[i];
			if (id >= last_id) break;
			T &f = container[id];
			f.type_id = type_id;
			f.has_p2d = true;
			f.p2d = features[i];
		}
		return true;
	}*/

	/** \brief Track features matching the given functor. If first_id >= 0 we call \e AddFeatures with the specified id range. */
	template<typename T, typename F>
	bool Track(IplImage *img, IplImage *mask, std::map<int,T> &container, F do_handle_test, int type_id=0, int first_id=-1, int last_id=-1)
	{
		// When first_id && last_id are < 0 then we don't add new features...
		if (first_id < 0) last_id = -1;
		// Update features to match the ones in the given container
		typename std::map<int,T>::iterator iter = container.begin();
		typename std::map<int,T>::iterator iter_end = container.end();
		feature_count = 0;
		for (;iter != iter_end; iter++) {
			T &f = iter->second;
			if (!do_handle_test(f)) continue;
			if (f.has_p2d != true) continue;
			f.has_p2d = false; // This is updated again to true if tracking succeeds
			features[feature_count] = f.p2d;
			ids[feature_count] = iter->first;
			feature_count++;
			if (feature_count == max_features) break;
		}
		// Purge if needed
		if (purge) {
			TrackerFeatures::Purge();
			purge=false;
		}
		if (first_id < 0) {
			// Track as usual (this will swap above features to prev_features)
			TrackHid(img, mask, false);
		} else {
			// Check that next_id is ok
			if (next_id < first_id)	next_id = first_id;
			if (next_id > last_id) return false; // TODO: Make some better solution for this
			// Track as usual (this will swap above features to prev_features)
			TrackHid(img, mask, true);
		}
		// Update the container to have the updated features
		for (int i=0; i<feature_count; i++) {
			int id = ids[i];
			if (last_id >= 0 && id >= last_id) break;
			T &f = container[id];
			f.type_id = type_id;
			f.has_p2d = true;
			f.p2d = features[i];
		}
		return true;
	}

	/** \brief add features to the previously tracked frame if there are less than min_features */
	template<typename T>
	bool AddFeatures(std::map<int,T> &container, int type_id=0, int first_id=0, int last_id=65535)
	{
		if (TrackerFeatures::AddFeatures() == 0) return false;
		// Update the container to have the updated features
		for (int i=0; i<feature_count; i++) {
			int id = ids[i];
			if (last_id >= 0 && id >= last_id) break;
			T &f = container[id];
			f.type_id = type_id;
			f.has_p2d = true;
			f.p2d = features[i];
		}
		return true;
	}

	/** \brief Erases the items matching with \e type_id having \e has_p2d == false . If \e type_id == -1 doesn't test the type. */
	template<typename T>
	int EraseNonTracked(std::map<int,T> &container, int type_id=-1)
	{
		DoEraseTest<T> do_erase_test(true, false, type_id);
		return EraseItemsEC(container, do_erase_test);
	}
	/** \brief Purge features that are considerably closer than the defined min_distance. 
	 *
	 * Note, that we always try to maintain the smaller id's assuming that they are older ones
	 */
	void Purge() { purge=true; }
	void Reset() { throw alvar::AlvarException("Method not supported"); }
	double Reset(IplImage *img, IplImage *mask) { throw alvar::AlvarException("Method not supported"); }
	bool DelFeature(int index) { throw alvar::AlvarException("Method not supported"); }
	bool DelFeatureId(int id) { throw alvar::AlvarException("Method not supported"); }
};

/** \brief Version of \e Camera using external container */
class ALVAR_EXPORT CameraEC : public Camera {
public:
	/** \brief Undistort the items with matching \e type_id */
	template<typename T>
	void Undistort(std::map<int,T> &container, int type_id=-1) {
		DoHandleTest<T> do_handle_test_default(type_id);
		return Undistort(container, do_handle_test_default);
	}
	/** \brief Undistort the items matching the given functor */
	template<typename T, typename F>
	void Undistort(std::map<int,T> &container, F &do_handle_test) {
		typename std::map<int,T>::iterator iter = container.begin();
		typename std::map<int,T>::iterator iter_end = container.end();
		for (;iter != iter_end; iter++) {
			T &f = iter->second;
			if (!do_handle(f)) continue;
			if (f.has_p2d) Undistort(f.p2d);
		}
	}
	/** \brief Distort the items with matching \e type_id */
	template<typename T>
	void Distort(std::map<int,T> &container, int type_id=-1) {
		DoHandleTest<T> do_handle_test_default(type_id);
		return Distort(container, do_handle_test_default);
	}
	/** \brief Distort the items matching the given functor */
	template<typename T, typename F>
	void Distort(std::map<int,T> &container, F &do_handle_test) {
		typename std::map<int,T>::iterator iter = container.begin();
		typename std::map<int,T>::iterator iter_end = container.end();
		for (;iter != iter_end; iter++) {
			T &f = iter->second;
			if (!do_handle(f)) continue;
			if (f.has_p2d) Distort(f.p2d);
		}
	}
	/** \brief Calculate the \e pose using the items with matching \e type_id */
	template<typename T>
	bool CalcExteriorOrientation(std::map<int,T> &container, Pose *pose, int type_id=-1) {
		DoHandleTest<T> do_handle_test_default(type_id);
		return CalcExteriorOrientation(container, pose, do_handle_test_default);
	}
	/** \brief Calculate the \e pose using the items matching the given functor */
	template<typename T, typename F>
	bool CalcExteriorOrientation(std::map<int,T> &container, Pose *pose, F do_handle_test) {
		int count_points = container.size();
		if(count_points < 4) return false;
		CvMat* world_points = cvCreateMat(count_points, 1, CV_32FC3);
		CvMat* image_observations = cvCreateMat(count_points*2, 1, CV_32FC2);
		typename std::map<int,T>::iterator iter = container.begin();
		typename std::map<int,T>::iterator iter_end = container.end();
		int ind = 0;
		for (;iter != iter_end; iter++) {
			T &f = iter->second;
			if (!do_handle_test(f)) continue;
			if (f.has_p2d && f.has_p3d) {
				world_points->data.fl[ind*3+0] = (float)f.p3d.x;
				world_points->data.fl[ind*3+1] = (float)f.p3d.y;
				world_points->data.fl[ind*3+2] = (float)f.p3d.z;
				image_observations->data.fl[ind*2+0]  = (float)f.p2d.x;
				image_observations->data.fl[ind*2+1]  = (float)f.p2d.y;
				ind++;
			}
		}
		if (ind<4) return false;

		CvRect r; r.x = 0; r.y = 0; r.height = ind; r.width = 1;
		CvMat world_points_sub;
		cvGetSubRect(world_points, &world_points_sub, r);
		CvMat image_observations_sub;
		cvGetSubRect(image_observations, &image_observations_sub, r);

		bool ret = Camera::CalcExteriorOrientation(&world_points_sub, &image_observations_sub, pose);

		cvReleaseMat(&image_observations);
		cvReleaseMat(&world_points);

		return ret;		
	}
	/** \brief Update the \e pose using the items with matching \e type_id */
	template<typename T>
	bool UpdatePose(std::map<int,T> &container, Pose *pose, int type_id=-1, std::map<int,double> *weights=0) {
		DoHandleTest<T> do_handle_test_default(type_id);
		return UpdatePose(container, pose, do_handle_test_default, weights);
	}
	/** \brief Update the rotation in \e pose using the items with matching \e type_id */
	template<typename T>
	bool UpdateRotation(std::map<int,T> &container, Pose *pose, int type_id=-1) {
		DoHandleTest<T> do_handle_test_default(type_id);
		return UpdateRotation(container, pose, do_handle_test_default);
	}
	/** \brief Update the rotation in \e pose using the items  matching the given functor */
	template<typename T, typename F>
	bool UpdateRotation(std::map<int,T> &container, Pose *pose, F do_handle_test) {
		int count_points = container.size();
		if(count_points < 6) return false;

		CvMat* world_points = cvCreateMat(count_points, 1, CV_64FC3);
		CvMat* image_observations = cvCreateMat(count_points*2, 1, CV_64F); // [u v u v u v ...]'

		int ind = 0;
		typename std::map<int,T>::iterator iter = container.begin();
		typename std::map<int,T>::iterator iter_end = container.end();
		for (;iter != iter_end; iter++) {
			T &f = iter->second;
			if (!do_handle_test(f)) continue;
			if (f.has_p2d && f.has_p3d) {
				world_points->data.db[ind*3+0] = f.p3d.x;
				world_points->data.db[ind*3+1] = f.p3d.y;
				world_points->data.db[ind*3+2] = f.p3d.z;
				image_observations->data.db[ind*2+0] = f.p2d.x;
				image_observations->data.db[ind*2+1] = f.p2d.y;
				ind++;
			}
		}
		if (ind < 6) return false;

		CvRect r; r.x = 0; r.y = 0; r.height = ind; r.width = 1;
		CvMat world_points_sub;
		cvGetSubRect(world_points, &world_points_sub, r);

		r.height = 2*ind;
		CvMat image_observations_sub;
		cvGetSubRect(image_observations, &image_observations_sub, r);

		bool ret = UpdateRotation(&world_points_sub, &image_observations_sub, pose);

		cvReleaseMat(&image_observations);
		cvReleaseMat(&world_points);

		return ret;		
	}	
	/** \brief Update the \e pose using the items matching the given functor */
	template<typename T, typename F>
	bool UpdatePose(std::map<int,T> &container, Pose *pose, F do_handle_test, std::map<int,double> *weights=0) {
		int count_points = container.size();
		if(count_points < 4) return false; // Note, UpdatePose calls CalcExteriorOrientation if 4 or 5 points

		CvMat* world_points = cvCreateMat(count_points, 1, CV_64FC3);
		CvMat* image_observations = cvCreateMat(count_points*2, 1, CV_64F); // [u v u v u v ...]'
		CvMat* w = 0;
		if(weights) w = cvCreateMat(count_points*2, 1, CV_64F);

		int ind = 0;
		typename std::map<int,T>::iterator iter = container.begin();
		typename std::map<int,T>::iterator iter_end = container.end();
		for (;iter != iter_end; iter++) {
			T &f = iter->second;
			if (!do_handle_test(f)) continue;
			if (f.has_p2d && f.has_p3d) {
				world_points->data.db[ind*3+0] = f.p3d.x;
				world_points->data.db[ind*3+1] = f.p3d.y;
				world_points->data.db[ind*3+2] = f.p3d.z;
				image_observations->data.db[ind*2+0] = f.p2d.x;
				image_observations->data.db[ind*2+1] = f.p2d.y;

				if(weights) 
					w->data.db[ind*2+1] = w->data.db[ind*2+0] = (*weights)[iter->first];
				ind++;
			}
		}
		if (ind < 4) return false; // Note, UpdatePose calls CalcExteriorOrientation if 4 or 5 points

		CvRect r; r.x = 0; r.y = 0; r.height = ind; r.width = 1;
		CvMat world_points_sub;
		cvGetSubRect(world_points, &world_points_sub, r);

		r.height = 2*ind;
		CvMat image_observations_sub;
		cvGetSubRect(image_observations, &image_observations_sub, r);

		bool ret;
		if (weights) {
			CvMat w_sub;
			cvGetSubRect(w, &w_sub, r);
			ret = UpdatePose(&world_points_sub, &image_observations_sub, pose, &w_sub);
		}
		else 
			ret = UpdatePose(&world_points_sub, &image_observations_sub, pose);

		cvReleaseMat(&image_observations);
		cvReleaseMat(&world_points);
		if(w) cvReleaseMat(&w);

		return ret;		
	}

	/** \brief Projects \e p3d in the items matching with \e type_id into \e projected_p2d . If \e type_id == -1 doesn't test the type. */
	template<typename T>
	float Reproject(std::map<int,T> &container, Pose *pose, int type_id=-1, bool needs_has_p2d=false, bool needs_has_p3d=false, float average_outlier_limit=0.f)
	{
		DoHandleTest<T> do_handle_test(type_id, needs_has_p2d, needs_has_p3d);
		return Reproject(container, pose, do_handle_test, average_outlier_limit);
	}
	/** \brief Projects \e p3d in the items matching with the given functor */
	template<typename T, typename F>
	float Reproject(std::map<int,T> &container, Pose *pose, F do_handle_test, float average_outlier_limit=0.f)
	{
		float reprojection_sum=0.f;
		int   reprojection_count=0;
		float limit_sq = average_outlier_limit*average_outlier_limit;
		typename std::map<int,T>::iterator iter = container.begin();
		typename std::map<int,T>::iterator iter_end = container.end();
		for(;iter != iter_end;iter++) {
			T &f = iter->second;
			if (!do_handle_test(f)) continue;
			Camera::ProjectPoint(iter->second.p3d, pose, iter->second.projected_p2d);
			Camera::ProjectPoint(iter->second.p3d_sh, pose, iter->second.projected_p2d_sh);

			// TODO: Now this is calculated in several places (should this distance be included in ExternalContainer structure?
			float dist_sq = (f.p2d.x-f.projected_p2d.x)*(f.p2d.x-f.projected_p2d.x);
			dist_sq       += (f.p2d.y-f.projected_p2d.y)*(f.p2d.y-f.projected_p2d.y);
			if ((limit_sq == 0) || (dist_sq < limit_sq)) {
				reprojection_sum += sqrt(dist_sq);
				reprojection_count++;
			}
		}
		if (reprojection_count == 0) return 0.f;
		return reprojection_sum/reprojection_count;
	}

	/** \brief Erases the items matching with \e type_id having large reprojection error. 
	 * If \e type_id == -1 doesn't test the type. 
	 * If \e pose is given calls \e Reproject() internally -- otherwise assumes it has been called beforehands. 
	 */
	template<typename T>
	int EraseUsingReprojectionError(std::map<int,T> &container, float reprojection_error_limit, int type_id=-1, Pose *pose = 0)
	{
		if (pose) Reproject(container, pose, type_id, false, false);
		DoEraseTest<T> do_erase_test(reprojection_error_limit, false, false, type_id) ;
		return EraseItemsEC(container, do_erase_test);
	}

	/** \brief Update pose rotations based on new observations */
	bool UpdateRotation(const CvMat* object_points, CvMat* image_points, Pose *pose);

	/** \brief Update pose rotations (in rodriques&tra) based on new observations */
	bool UpdateRotation(const CvMat* object_points, CvMat* image_points, CvMat *rot, CvMat *tra);

	/** \brief Update existing pose based on new observations */
	bool UpdatePose(const CvMat* object_points, CvMat* image_points, Pose *pose, CvMat *weights=0);

	/** \brief Update existing pose (in rodriques&tra) based on new observations */
	bool UpdatePose(const CvMat* object_points, CvMat* image_points, CvMat *rodriques, CvMat *tra, CvMat *weights=0);

	/** \brief Reconstruct 3D point using two camera poses and corresponding undistorted image feature positions */
	bool ReconstructFeature(const Pose *pose1, const Pose *pose2, const CvPoint2D32f *u1, const CvPoint2D32f *u2, CvPoint3D32f *p3d, double limit);

	/** \brief Get 3D-coordinate for 2D feature on the plane defined by the pose (z == 0) */ 
	void Get3dOnPlane(const Pose *pose, CvPoint2D32f p2d, CvPoint3D32f &p3d);

	/** \brief Get 3D-coordinate for 2D feature assuming certain depth */
	void Get3dOnDepth(const Pose *pose, CvPoint2D32f p2d, float depth, CvPoint3D32f &p3d);
};

/** \brief Calculate the index used in external container map for specified \e marker_id */
inline int MarkerIdToContainerId(int marker_id, int corner_id, int first_id=0, int last_id=65535) {
	int id = first_id + marker_id*4 + corner_id;
	if (id > last_id) return -1;
	return id;
}

/** \brief Version of \e MarkerDetector using external container */
template<class M>
class MarkerDetectorEC : public MarkerDetector<M> {
protected:
	template<typename T>
	bool PreDetect(std::pair<const int,T> &p, int type_id) {
		return PreDetect(p.second, type_id);
	}
	template<typename T>
	bool PreDetect(T &p, int type_id) {
		if (p.type_id != type_id) return false;
		p.has_p2d = false; // This is updated again to true if tracking succeeds
		return true;
	}
public:
	int GetId(int marker_id, int corner_id, int first_id=0, int last_id=65535) {
		return MarkerIdToContainerId(marker_id, corner_id, first_id, last_id);
	}

	/** \brief Detect markers in the image and fill in their 2D-positions in the given external container */
	template<typename T>
	int Detect(IplImage *image,
			   Camera *cam,
			   std::map<int,T> &container,
			   int type_id=0,
			   int first_id=0,
			   int last_id=65535,
			   bool track=false,
			   bool visualize=false,
			   double max_new_marker_error=0.08,
			   double max_track_error=0.2,
			   LabelingMethod labeling_method=CVSEQ)
	{
		int ret;

		// The existing markers in the container are marked to not have valid p2d (has_p2d=false)
		typename std::map<int,T>::iterator iter = container.begin();
		typename std::map<int,T>::iterator iter_end = container.end();
		for (;iter != iter_end; iter++) {
			T &f = iter->second;
			if (f.type_id != type_id) continue;
			f.has_p2d = false; // This is updated again to true if tracking succeeds
		}

		// Detect without making the unnecessary pose update
		ret = MarkerDetector<M>::Detect(image, cam, track, visualize, max_new_marker_error, max_track_error, labeling_method, false);

		// Fill in the detected markers to the container
		for (size_t i=0; i<MarkerDetector<M>::markers->size(); i++) {
			M &m = (*(MarkerDetector<M>::markers))[i];
			for (int corner=0; corner<4; corner++) {
				int id = MarkerIdToContainerId(m.GetId(), corner, first_id, last_id);
				if (id != -1) {
					T &f = container[id];
					f.type_id = type_id;
					f.has_p2d = true;
					f.p2d.x = float(m.marker_corners_img[corner].x);
					f.p2d.y = float(m.marker_corners_img[corner].y);
				}
			}
		}
		return ret;
	}

	/** \brief Fill in 3D-coordinates for \e marker_id marker corners using \e edge_length and \e pose */
	template<typename T>
	void MarkerToEC(std::map<int,T> &container, int marker_id, double edge_length, Pose &pose, int type_id=0, int first_id=0,int last_id=65535) {
		CvMat *m3 = cvCreateMat(4,4,CV_64F); cvSetIdentity(m3);
		pose.GetMatrix(m3);

		for(size_t corner = 0; corner < 4; ++corner)
		{
			// TODO: This should be exactly the same as in Marker class.
			//       Should we get the values from there somehow?
			double X_data[4] = {0, 0, 0, 1};
			if (corner == 0) { 
				X_data[0] = -0.5*edge_length;
				X_data[1] = -0.5*edge_length;
			} else if (corner == 1) {
				X_data[0] = +0.5*edge_length;
				X_data[1] = -0.5*edge_length;
			} else if (corner == 2) {
				X_data[0] = +0.5*edge_length;
				X_data[1] = +0.5*edge_length;
			} else if (corner == 3) {
				X_data[0] = -0.5*edge_length;
				X_data[1] = +0.5*edge_length;
			}

			CvMat X  = cvMat(4, 1, CV_64F, X_data);
			cvMatMul(m3, &X, &X);

			int id = MarkerIdToContainerId(marker_id, corner, first_id, last_id);
			T &f = container[id];
			f.type_id = type_id;
			f.p3d.x = float(X_data[0] / X_data[3]);
			f.p3d.y = float(X_data[1] / X_data[3]);
			f.p3d.z = float(X_data[2] / X_data[3]);
			f.has_p3d = true;
		}
		cvReleaseMat(&m3);
	}
};

/** \brief Version of \e MultiMarker using external container */
class MultiMarkerEC : public MultiMarker {
public:
	/** \brief Fill in 3D-coordinates for marker corners to the given container */
	template<typename T>
	bool MarkersToEC(std::map<int,T> &container, int type_id=0, int first_id=0,int last_id=65535) {
		bool ret = false;
		for (size_t i = 0; i < marker_indices.size(); i++) {
			if (marker_status[i] == 0) continue; // Skip the ones not in point cloud 
			int marker_id = marker_indices[i];
			for (int corner = 0; corner < 4; corner++) {
				int id = MarkerIdToContainerId(marker_id, corner, first_id, last_id);
				if (id != -1) {
					int pc_index = 	pointcloud_index(marker_id, corner);
					T &f = container[id];
					f.type_id = type_id;
					f.p3d.x = float(pointcloud[pc_index].x);
					f.p3d.y = float(pointcloud[pc_index].y);
					f.p3d.z = float(pointcloud[pc_index].z);
					f.has_p3d = true;
					ret = true;
				}
			}
		}
		return ret;
	}

	template<typename T>
	bool MarkersFromEC(std::map<int,T> &container, int type_id=0, int first_id=0,int last_id=65535) {
		// TODO...
		return false;
	}

	template<typename T>
	bool Save(std::map<int,T> &container, const char* fname, FILE_FORMAT format = FILE_FORMAT_DEFAULT, int type_id=0, int first_id=0,int last_id=65535) {
		if (!MarkersFromEC(container, type_id, first_id, last_id)) return false;
		if (!MultiMarker::Save(fname, format)) return false;
		return true;
	}

	/** \brief Fill in 3D-coordinates for marker corners to the given container using save \e MultiMarker file */
	template<typename T>
	bool Load(std::map<int,T> &container, const char* fname, FILE_FORMAT format = FILE_FORMAT_DEFAULT, int type_id=0, int first_id=0,int last_id=65535) {
		if (!MultiMarker::Load(fname, format)) return false;
		return MarkersToEC(container, type_id, first_id, last_id);
	}
};
}

#endif


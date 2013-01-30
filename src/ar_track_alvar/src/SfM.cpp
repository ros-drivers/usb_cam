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

#include "SfM.h"

namespace alvar {

class CameraMoves
{
	bool is_initialized;
	double prev[3];

public:

	CameraMoves()
	{
		is_initialized = false;
		memset(prev, 0, 3*sizeof(double));
	}

	bool UpdateDistance(Pose* pose, double limit=10)
	{
		double trad[3]; CvMat tra = cvMat(3, 1, CV_64F, trad);
		Pose p = *pose;
		p.Invert();
		p.GetTranslation(&tra);
		if(is_initialized == false)
		{
			memcpy(prev, trad, 3*sizeof(double));
			is_initialized = true;
			return false;
		}
		double dist = (prev[0]-trad[0])*(prev[0]-trad[0]) + 
						(prev[1]-trad[1])*(prev[1]-trad[1]) + 
						(prev[2]-trad[2])*(prev[2]-trad[2]);
		if(dist > limit)
		{
			memcpy(prev, trad, 3*sizeof(double));
			return true;
		}
		return false;		
	}
};
CameraMoves moving;

void SimpleSfM::Reset(bool reset_also_triangulated) {
	pose_ok = false;
	container = container_reset_point;
	if (reset_also_triangulated) {
		container_triangulated = container_triangulated_reset_point;
	}
	pose_difference.Reset();
	pose_difference.Mirror(false, true, true);
}

void SimpleSfM::SetResetPoint() {
	container_reset_point = container;
	container_triangulated_reset_point = container_triangulated;
}

void SimpleSfM::Clear() {
	container.clear();
}

CameraEC *SimpleSfM::GetCamera() { return &cam; }

Pose *SimpleSfM::GetPose() { return &pose; }

bool SimpleSfM::AddMultiMarker(const char *fname, FILE_FORMAT format/* = FILE_FORMAT_XML*/) {
	MultiMarkerEC mm;
	return mm.Load(container, fname, format, 0, 0,1023);
}

bool SimpleSfM::AddMultiMarker(MultiMarkerEC *mm) {
	return mm->MarkersToEC(container, 0, 0, 1023);
}

void SimpleSfM::AddMarker(int marker_id, double edge_length, Pose &pose) {
	marker_detector.MarkerToEC(container, marker_id, edge_length, pose, 0, 0, 1023);
}

float PointVectorFromCamera(CvPoint3D32f p3d, CvPoint3D32f &p3d_vec, Pose *camera_pose) {
	double pd[16], v[4] = {0,0,0,1};	
	CvMat Pi = cvMat(4, 4, CV_64F, pd);
	CvMat V = cvMat(4, 1, CV_64F, v);

	// Camera location in marker coordinates
	camera_pose->GetMatrix(&Pi);
	cvInv(&Pi, &Pi);
	cvMatMul(&Pi, &V, &V);
	v[0] /= v[3];
	v[1] /= v[3];
	v[2] /= v[3];

	// Vector from camera-point to the 3D-point in marker coordinates
	p3d_vec.x = float(p3d.x - v[0]);
	p3d_vec.y = float(p3d.y - v[1]);
	p3d_vec.z = float(p3d.z - v[2]);

	return std::sqrt(p3d_vec.x*p3d_vec.x + p3d_vec.y*p3d_vec.y + p3d_vec.z*p3d_vec.z);
}

void CreateShadowPoint(CvPoint3D32f &p3d_sh, CvPoint3D32f p3d, CameraEC *cam, Pose *camera_pose, float parallax_length, float triangulate_angle) {
	// Vector from camera-point to the 3D-point in marker coordinates
	float l = PointVectorFromCamera(p3d, p3d_sh, camera_pose);

	// Figure out a suitable matching shadow point distance
	CvPoint2D32f p2d;
	CvPoint3D32f p3d2;
	cam->ProjectPoint(p3d, camera_pose, p2d);
	p2d.x += parallax_length;
	cam->Get3dOnDepth(camera_pose, p2d, l, p3d2);
	p3d2.x -= p3d.x;
	p3d2.y -= p3d.y;
	p3d2.z -= p3d.z;
	float pl = std::sqrt(p3d2.x*p3d2.x + p3d2.y*p3d2.y + p3d2.z*p3d2.z);
	float shadow_point_dist = float(pl / std::abs(tan(triangulate_angle*3.1415926535/180.))); // How far we need the shadow point to have the parallax limit match

	// Create the shadow point
	p3d_sh.x /= l;
	p3d_sh.y /= l;
	p3d_sh.z /= l;
	p3d_sh.x *= shadow_point_dist;
	p3d_sh.y *= shadow_point_dist;
	p3d_sh.z *= shadow_point_dist;
	p3d_sh.x += p3d.x;
	p3d_sh.y += p3d.y;
	p3d_sh.z += p3d.z;

	//std::cout<<"p  : "<<p3d.x<<","<<p3d.y<<","<<p3d.z<<std::endl;	
	//std::cout<<"psh: "<<p3d_sh.x<<","<<p3d_sh.y<<","<<p3d_sh.z<<std::endl;	
}

bool SimpleSfM::Update(IplImage *image, bool assume_plane, bool triangulate, float reproj_err_limit, float triangulate_angle) {
	const int min_triangulate=50, max_triangulate=150;
	const float plane_dist_limit = 100.f;
	const bool remember_3d_points = true;
	const float parallax_length = 10.f;
	const float parallax_length_sq = parallax_length*parallax_length;
	float reproj_err_limit_sq = reproj_err_limit*reproj_err_limit;
	std::map<int, Feature>::iterator  iter;
	std::map<int, Feature>::iterator iter_end = container.end();

	// #1. Detect marker corners and track features
	marker_detector.Detect(image, &cam, container, 0, 0, 1023, true, false);
	tf.Purge();
	tf.Track(image, 0, container, 1); // Track without adding features
	//tf.Track(image, 0, container, 1, 1024, 65535); // Track with adding features
	tf.EraseNonTracked(container, 1);

	// #2. Update pose
	if (!pose_ok) pose_ok = cam.CalcExteriorOrientation(container, &pose);
	pose_ok = cam.UpdatePose(container, &pose);
	if (!pose_ok) pose_ok = cam.UpdatePose(container, &pose, 0); // Only markers
	if (!pose_ok) return false;

	// #3. Reproject features and their shadows
	double err_markers  = cam.Reproject(container, &pose, 0, true, false, 100);
	if (err_markers > reproj_err_limit*2) { Reset(false); return false;}
	double err_features = cam.Reproject(container, &pose, 1, false, false, 100);
	if ((err_markers == 0) && (err_features > reproj_err_limit*2)) {
		// Error is so large, that we just are satisfied we have some pose
		// Don't triangulate and remove points based on this result
		return true;
	}

	// #4. Add previously triangulated features to container if they are visible...
	if (remember_3d_points) {
		IplImage *mask = tf.NewFeatureMask();
		iter = container_triangulated.begin();
		while(iter != container_triangulated.end()) {
			if (container.find(iter->first) == container.end()) {
				Camera *c = &cam;
				c->ProjectPoint(iter->second.p3d, &pose, iter->second.projected_p2d);
				if ((iter->second.projected_p2d.x >= 0) &&
					(iter->second.projected_p2d.y >= 0) &&
					(iter->second.projected_p2d.x < image->width) &&
					(iter->second.projected_p2d.y < image->height))
				{
					CvScalar s = cvGet2D(mask, int(iter->second.projected_p2d.y), int(iter->second.projected_p2d.x));
					if (s.val[0] == 0) {
						container[iter->first] = iter->second;
						container[iter->first].estimation_type = 3;
						container[iter->first].p2d = container[iter->first].projected_p2d;
					}
				}
			}
			iter++;
		}
	}

	// #5. Add other features to container
	// Assume them initially on marker plane if (assume_plane == true)
	tf.AddFeatures(container, 1, 1024, 0xffffff); //65535);
	if (assume_plane) {
		for (iter = container.begin(); iter != iter_end; iter++) {
			Feature &f = iter->second;
			//if (f.type_id == 0) continue;
			if (!f.has_p3d && f.estimation_type < 1) {
				//cam.Get3dOnPlane(&pose, f.p2d, f.p3d);

				//CvPoint3D32f p3d_vec;
				//float l = PointVectorFromCamera(f.p3d, p3d_vec, &pose);
				//if (l > plane_dist_limit) cam.Get3dOnDepth(&pose, f.p2d, plane_dist_limit, f.p3d);
				cam.Get3dOnDepth(&pose, f.p2d, plane_dist_limit, f.p3d);

				// TODO: Now we don't create a shadow point for plane points. This is because
				//       we don't want to remember them as 3d points in container_triangulated.
				//       We probably get the same benefit just by assuming that everything is on plane?
				//CreateShadowPoint(f.p3d_sh, f.p3d, &pose, shadow_point_dist);
				f.p3d_sh = f.p3d;

				f.has_p3d = true;
				f.estimation_type = 1;
			}
		}
	}

	// #6. Triangulate features with good parallax
	if (triangulate) {
		for (iter = container.begin(); iter != iter_end; iter++) {
			Feature &f = iter->second;
			//if (f.type_id == 0) continue;
			if (!f.has_p3d && !f.has_stored_pose) {
				f.pose1 = pose;
				f.p2d1 = f.p2d;
				f.has_stored_pose = true;
				cam.Get3dOnDepth(&pose, f.p2d, 8, f.p3d);
				CreateShadowPoint(f.p3d_sh, f.p3d, &cam, &pose, parallax_length, triangulate_angle);
			}
			if (!f.has_p3d && f.has_stored_pose && f.estimation_type < 2) {
				double dist_sh_sq = (f.projected_p2d_sh.x-f.projected_p2d.x)*(f.projected_p2d_sh.x-f.projected_p2d.x);
				dist_sh_sq       += (f.projected_p2d_sh.y-f.projected_p2d.y)*(f.projected_p2d_sh.y-f.projected_p2d.y);
				if (dist_sh_sq > parallax_length_sq) {
					CvPoint2D32f u1 = f.p2d1;
					CvPoint2D32f u2 = f.p2d;
					cam.Camera::Undistort(u1);
					cam.Camera::Undistort(u2);
					if(cam.ReconstructFeature(&f.pose1, &pose, &u1, &u2, &f.p3d, 10e-6)) {
						CreateShadowPoint(f.p3d_sh, f.p3d,  &cam, &pose, parallax_length, triangulate_angle);
						f.has_p3d = true;
					} else
						f.has_stored_pose = false;
					f.estimation_type = 2;
				}
			}
		}
	}

	// #7. Erase poor features
	cam.Reproject(container, &pose, 1, false, false, 100); // TODO: Why again...
	iter = container.begin();
	while(iter != container.end()) {
		bool iter_inc = true;
		if (iter->second.type_id > 0) {
			Feature &f = iter->second;
			if ((f.estimation_type == 0) || !f.has_p2d || !f.has_p3d);
			else {
				// Note that the projected_p2d needs to have valid content at this point
				double dist_sq = (f.p2d.x-f.projected_p2d.x)*(f.p2d.x-f.projected_p2d.x);
				dist_sq       += (f.p2d.y-f.projected_p2d.y)*(f.p2d.y-f.projected_p2d.y);

				if (f.estimation_type == 1) {
					if (dist_sq > (reproj_err_limit_sq)) f.has_p3d = false;
				} else if (f.estimation_type == 2) {
					if (dist_sq > (reproj_err_limit_sq)) {
						container.erase(iter++);
						iter_inc = false;
					}
				} else if (f.estimation_type == 3) {
					if (container_triangulated.size() < min_triangulate) {
						if (dist_sq > (reproj_err_limit_sq)) {
							container.erase(iter++);
							iter_inc = false;
						}
					} else {
						if (dist_sq > (reproj_err_limit_sq)) f.has_p3d = false;
						if (dist_sq > (reproj_err_limit_sq*2)) {
							std::map<int, Feature>::iterator iter_tmp;
							iter_tmp = container_triangulated.find(iter->first);
							if (iter_tmp != container_triangulated.end()) {
								container_triangulated.erase(iter_tmp);
							}
							container.erase(iter++);
							iter_inc = false;
						}
					}
				}
			}
		}
		if (iter_inc) iter++;
	}

	// #8. Remember good features that have little reprojection error while the parallax is noticeable
	if (remember_3d_points && (container_triangulated.size() < max_triangulate)) {
		iter = container.begin();
		while(iter != container.end()) {
			if ((iter->second.type_id > 0) &&
				(container_triangulated.find(iter->first) == container_triangulated.end()))
			{
				Feature &f = iter->second;
				double dist_sq = (f.p2d.x-f.projected_p2d.x)*(f.p2d.x-f.projected_p2d.x);
				dist_sq       += (f.p2d.y-f.projected_p2d.y)*(f.p2d.y-f.projected_p2d.y);
				double dist_sh_sq = (f.projected_p2d_sh.x-f.projected_p2d.x)*(f.projected_p2d_sh.x-f.projected_p2d.x);
				dist_sh_sq       += (f.projected_p2d_sh.y-f.projected_p2d.y)*(f.projected_p2d_sh.y-f.projected_p2d.y);
				if ((dist_sq < reproj_err_limit_sq) && (dist_sh_sq > parallax_length_sq)) { 
					container_triangulated[iter->first] = iter->second;
					f.estimation_type = 3;
				}
			}
			iter++;
			if (container_triangulated.size() >= max_triangulate) break;
		}
	}

	return true;
}

bool SimpleSfM::UpdateTriangulateOnly(IplImage *image) {
	marker_detector.Detect(image, &cam, container, 0, 0, 1023, true, false);
	tf.Purge();
	tf.Track(image, 0, container, 1, 1024, 65535);
	tf.EraseNonTracked(container, 1);
	if (!pose_ok) pose_ok = cam.CalcExteriorOrientation(container, &pose);
	else pose_ok = cam.UpdatePose(container, &pose);
	if(pose_ok) update_tri = moving.UpdateDistance(&pose, scale);
	std::map<int,Feature>::iterator iter = container.begin();
	std::map<int,Feature>::iterator iter_end = container.end();
	for(;iter != iter_end; iter++) {
		if (pose_ok && (iter->second.type_id == 1) && (!iter->second.has_stored_pose) && (!iter->second.has_p3d)) {
			iter->second.pose1 = pose;
			iter->second.p2d1 = iter->second.p2d;
			iter->second.has_stored_pose = true;
		}
	}
	iter = container.begin();
	for(;iter != iter_end; iter++)
	{
		Feature &f = iter->second;
		if(update_tri && f.has_stored_pose && !f.has_p3d) {
			f.tri_cntr++;
		}
		if(f.tri_cntr==3) {
			CvPoint2D32f u1 = f.p2d1;
			CvPoint2D32f u2 = f.p2d;
			cam.Camera::Undistort(u1);
			cam.Camera::Undistort(u2);
			if(cam.ReconstructFeature(&f.pose1, &pose, &u1, &u2, &f.p3d, 10e-6))
				f.has_p3d = true;
			else
				f.has_stored_pose = false;
			f.tri_cntr = 0;
		}
	}
	if (pose_ok) {
		double err_markers  = cam.Reproject(container, &pose, 0, true, false, 100);
		if(err_markers > 30) { Reset(false); return false;}
		double err_features = cam.Reproject(container, &pose, 1, false, false, 100);
		cam.EraseUsingReprojectionError(container, 30, 1);
	}
	return pose_ok;
}

bool SimpleSfM::UpdateRotationsOnly(IplImage *image) {
	int n_markers = marker_detector.Detect(image, &cam, container, 0, 0, 1023, true, false);
	static int n_markers_prev = 0;
	tf.Purge();
	tf.Track(image, 0, container, 1, 1024, 65535);
	tf.EraseNonTracked(container, 1);
	int type_id = -1;
	if(n_markers>=1) type_id = 0;
	if (n_markers==1)
	{
		pose_ok = cam.CalcExteriorOrientation(container, &pose, 0);
		if(pose_ok) pose_original = pose;
	}
	else if(n_markers>1)
	{
		if(pose_ok)
			pose_ok = cam.UpdatePose(container, &pose, 0);
		else
			pose_ok = cam.CalcExteriorOrientation(container, &pose, 0);
		if(pose_ok) pose_original = pose;
	}
	else //if(pose_ok) // Tracking going on...
	{
		if (n_markers_prev > 0) {
			pose_difference.Reset();
			pose_difference.Mirror(false, true, true);
			std::map<int,Feature>::iterator iter = container.begin();
			std::map<int,Feature>::iterator iter_end = container.end();
			for(;iter != iter_end; iter++) {
				if ((iter->second.type_id == 1) && (iter->second.has_p2d)) {
					CvPoint2D32f _p2d = iter->second.p2d;
					cam.Get3dOnDepth(&pose_difference, _p2d, 1, iter->second.p3d);
					iter->second.has_p3d = true;
				}
			}
		}
		cam.UpdateRotation(container, &pose_difference, 1);
		if(pose_ok)
		{
			double rot_mat[16];
			double gl_mat[16];
			pose_difference.GetMatrixGL(rot_mat);
			pose_original.GetMatrixGL(gl_mat);
			CvMat rot = cvMat(4, 4, CV_64F, rot_mat);// Rotation matrix (difference) from the tracker
			CvMat mod = cvMat(4, 4, CV_64F, gl_mat); // Original modelview matrix (camera location)
			cvMatMul(&mod, &rot, &rot);
			/*if(pose_ok)*/ pose.SetMatrixGL(rot_mat);
		}
		//pose_ok = true;
	}
	n_markers_prev = n_markers;
	if (pose_ok) {
		if (n_markers < 1) {
			std::map<int,Feature>::iterator iter = container.begin();
			std::map<int,Feature>::iterator iter_end = container.end();
			for(;iter != iter_end; iter++) {
				if ((iter->second.type_id == 1) && (!iter->second.has_p3d) && (iter->second.has_p2d)) {
					CvPoint2D32f _p2d = iter->second.p2d;
					cam.Get3dOnDepth(&pose_difference, _p2d, 1, iter->second.p3d);
					iter->second.has_p3d = true;
				}
			}
		}

		double err_markers  = cam.Reproject(container, &pose, 0, true, false, 100);
		if(err_markers > 10) { Reset(false); return false;}
		double err_features = cam.Reproject(container, &pose_difference, 1, false, false, 100);
		cam.EraseUsingReprojectionError(container, 10, 1);
	}
	return pose_ok;
}

void SimpleSfM::Draw(IplImage *rgba) {
	if(markers_found) return;
	std::map<int,Feature>::iterator iter = container.begin();
	std::map<int,Feature>::iterator iter_end = container.end();
	for(;iter != iter_end;iter++) {
		Feature &f = iter->second;
		if (f.has_p2d) {
			int r=0, g=0, b=0, rad=1;
			if (f.type_id == 0)              {r=255; g=0; b=0;}
			else if (f.estimation_type == 0) {
				if (f.has_stored_pose) {r=255; g=0; b=255;}
				else                   {r=0; g=255; b=255;}
			}
			else if (f.estimation_type == 1) {r=0; g=255; b=0;}
			else if (f.estimation_type == 2) {r=255; g=0; b=255;}
			else if (f.estimation_type == 3) {r=0; g=0; b=255;}
			if (f.has_p3d) rad=5;
			else rad = f.tri_cntr+1;

			cvCircle(rgba, cvPointFrom32f(f.p2d), rad, CV_RGB(r,g,b));
			if (pose_ok) {
				// The shadow point line
				if (f.type_id > 0 && f.estimation_type < 3 && f.p3d_sh.x != 0.f) {
					cvLine(rgba, cvPointFrom32f(f.projected_p2d), cvPointFrom32f(f.projected_p2d_sh), CV_RGB(0,255,0));
				}
				// Reprojection error
				if (f.has_p3d) {
					cvLine(rgba, cvPointFrom32f(f.p2d), cvPointFrom32f(f.projected_p2d), CV_RGB(255,0,255));
				}
			}
			//if (pose_ok && f.has_p3d) {
			//	cvLine(rgba, cvPointFrom32f(f.p2d), cvPointFrom32f(f.projected_p2d), CV_RGB(255,0,255));
			//}
			//if (f.type_id == 0) {
			//	int id = iter->first;
			//	cvCircle(rgba, cvPointFrom32f(f.p2d), 7, CV_RGB(255,0,0));
			//} else {
			//	int id = iter->first-1024+1;
			//	if (f.has_p3d) { 
			//		cvCircle(rgba, cvPointFrom32f(f.p2d), 5, CV_RGB(0,255,0));
			//	}
			//	else if (f.has_stored_pose) 
			//		cvCircle(rgba, cvPointFrom32f(f.p2d), f.tri_cntr+1, CV_RGB(255,0,255));
			//	else 
			//		cvCircle(rgba, cvPointFrom32f(f.p2d), 5, CV_RGB(0,255,255));
			//}
		}
	}
}

} // namespace alvar


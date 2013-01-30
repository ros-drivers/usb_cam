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

#include "Alvar.h"
#include "MultiMarker.h"
#include "FileFormatUtils.h"
#include <fstream>

using namespace std;

namespace alvar {
using namespace std;

int MultiMarker::pointcloud_index(int marker_id, int marker_corner, bool add_if_missing /*=false*/) {
	return (get_id_index(marker_id ,add_if_missing)*4)+marker_corner;
}

int MultiMarker::get_id_index(int id, bool add_if_missing /*=false*/)
{
	for(size_t i = 0; i < marker_indices.size(); ++i) {
		if(marker_indices.at(i) == id)
			return (int) i;
	}
	if (!add_if_missing) return -1;
	marker_indices.push_back(id);
	marker_status.push_back(0);
	return (marker_indices.size()-1);
}

void MultiMarker::Reset()
{
	fill(marker_status.begin(), marker_status.end(), 0);
	pointcloud.clear();
}

bool MultiMarker::SaveXML(const char* fname) {
	TiXmlDocument document;
	document.LinkEndChild(new TiXmlDeclaration("1.0", "UTF-8", "no"));
	document.LinkEndChild(new TiXmlElement("multimarker"));
	TiXmlElement *xml_root = document.RootElement();

	int n_markers = marker_indices.size();
	xml_root->SetAttribute("markers", n_markers);

	for(int i = 0; i < n_markers; ++i) {
		TiXmlElement *xml_marker = new TiXmlElement("marker");
		xml_root->LinkEndChild(xml_marker);

		xml_marker->SetAttribute("index", marker_indices[i]);
		xml_marker->SetAttribute("status", marker_status[i]);

		for(int j = 0; j < 4; ++j) {
			TiXmlElement *xml_corner = new TiXmlElement("corner");
			xml_marker->LinkEndChild(xml_corner);
			CvPoint3D64f X = pointcloud[pointcloud_index(marker_indices[i], j)];
			xml_corner->SetDoubleAttribute("x", X.x);
			xml_corner->SetDoubleAttribute("y", X.y);
			xml_corner->SetDoubleAttribute("z", X.z);
		}
	}
	return document.SaveFile(fname);
}

bool MultiMarker::SaveText(const char* fname) {
	size_t n_markers = marker_indices.size();

	fstream file_op(fname, ios::out);

	file_op<<n_markers<<endl;

	file_op<<endl;

	for(size_t i = 0; i < n_markers; ++i)
		file_op<<marker_indices[i]<<endl;

	file_op<<endl;

	for(size_t i = 0; i < n_markers; ++i)
		file_op<<marker_status[i]<<endl;

	file_op<<endl;

	for(size_t i = 0; i < n_markers; ++i)
		for(size_t j = 0; j < 4; ++j)
		{
			CvPoint3D64f X = pointcloud[pointcloud_index(marker_indices[i], j)];
			file_op<<X.x<<" "<<X.y<<" "<<X.z<<endl;
			
		}
	file_op.close();

	return true;
}

bool MultiMarker::Save(const char* fname, FILE_FORMAT format) {
	switch (format) {
		case FILE_FORMAT_XML:
			return SaveXML(fname);
		case FILE_FORMAT_TEXT:
		case FILE_FORMAT_DEFAULT:
			return SaveText(fname);
		default:
			return false;
	}
}

bool MultiMarker::LoadXML(const char* fname) {
	TiXmlDocument document;
	if (!document.LoadFile(fname)) return false;
	TiXmlElement *xml_root = document.RootElement();

	int n_markers;
	if (xml_root->QueryIntAttribute("markers", &n_markers) != TIXML_SUCCESS) return false;

	pointcloud.clear();
	marker_indices.resize(n_markers);
	marker_status.resize(n_markers);

	TiXmlElement *xml_marker = xml_root->FirstChildElement("marker");
	for(int i = 0; i < n_markers; ++i) {
		if (!xml_marker) return false;

		int index, status;
		if (xml_marker->QueryIntAttribute("index", &index) != TIXML_SUCCESS) return false;
		if (xml_marker->QueryIntAttribute("status", &status) != TIXML_SUCCESS) return false;
		marker_indices[i] = index;
		marker_status[i] = status;
		if(i==0) master_id = index;

		TiXmlElement *xml_corner = xml_marker->FirstChildElement("corner");
		for(int j = 0; j < 4; ++j) {
			if (!xml_corner) return false;

			CvPoint3D64f X;
			if (xml_corner->QueryDoubleAttribute("x", &X.x) != TIXML_SUCCESS) return false;
			if (xml_corner->QueryDoubleAttribute("y", &X.y) != TIXML_SUCCESS) return false;
			if (xml_corner->QueryDoubleAttribute("z", &X.z) != TIXML_SUCCESS) return false;
			pointcloud[pointcloud_index(marker_indices[i], j)] = X;

			xml_corner = (TiXmlElement*)xml_corner->NextSibling("corner");
		}

		xml_marker = (TiXmlElement*)xml_marker->NextSibling("marker");
	}
	return true;
}

bool MultiMarker::LoadText(const char* fname) {
	fstream file_op(fname, ios::in);

    if (!file_op) {
        return false;
    }

	size_t n_markers;
	file_op>>n_markers;

	pointcloud.clear();
	marker_indices.resize(n_markers);
	marker_status.resize(n_markers);

	for(size_t i = 0; i < n_markers; ++i){
		file_op>>marker_indices[i];
	}

	for(size_t i = 0; i < n_markers; ++i){
		file_op>>marker_status[i];
	}

	for(size_t i = 0; i < n_markers; ++i)
		for(size_t j = 0; j < 4; ++j)
		{
			CvPoint3D64f X;
			file_op>>X.x;
			file_op>>X.y;
			file_op>>X.z;
			pointcloud[pointcloud_index(marker_indices[i], j)] = X;
		}

	file_op.close();

	return true;
}

bool MultiMarker::Load(const char* fname, FILE_FORMAT format) {
	switch (format) {
		case FILE_FORMAT_XML:
			return LoadXML(fname);
		case FILE_FORMAT_TEXT:
		case FILE_FORMAT_DEFAULT:
			return LoadText(fname);
		default:
			return false;
	}
}

MultiMarker::MultiMarker(vector<int>& indices)
{
	marker_indices.resize(indices.size());
	copy(indices.begin(), indices.end(), marker_indices.begin());
	marker_status.resize(indices.size());
	fill(marker_status.begin(), marker_status.end(), 0);
}

void MultiMarker::PointCloudReset() {
	pointcloud.clear();
}

void MultiMarker::PointCloudCorners3d(double edge_length, Pose &pose, CvPoint3D64f corners[4]) {
	// Transformation from origin to current marker
	CvMat *m3 = cvCreateMat(4,4,CV_64F); cvSetIdentity(m3);
	pose.GetMatrix(m3);

	for(size_t j = 0; j < 4; ++j)
	{
		// TODO: This should be exactly the same as in Marker class.
		//       Should we get the values from there somehow?
		double X_data[4] = {0, 0, 0, 1};
		if (j == 0) { 
			int zzzz=2;			
			//X_data[0] = -0.5*edge_length;
			//X_data[1] = -0.5*edge_length;
		} else if (j == 1) {
			X_data[0] = +0.5*edge_length;
			X_data[1] = -0.5*edge_length;
		} else if (j == 2) {
			X_data[0] = +0.5*edge_length;
			X_data[1] = +0.5*edge_length;
		} else if (j == 3) {
			X_data[0] = -0.5*edge_length;
			X_data[1] = +0.5*edge_length;
		}

		CvMat X  = cvMat(4, 1, CV_64F, X_data);
		cvMatMul(m3, &X, &X);

		corners[j].x = X_data[0] / X_data[3];
		corners[j].y = X_data[1] / X_data[3];
		corners[j].z = X_data[2] / X_data[3];
	}
	cvReleaseMat(&m3);
}

void MultiMarker::PointCloudAdd(int marker_id, double edge_length, Pose &pose) {
	CvPoint3D64f corners[4];
	PointCloudCorners3d(edge_length, pose, corners);
	for(size_t j = 0; j < 4; ++j) {
		pointcloud[pointcloud_index(marker_id, j, true)] = corners[j];
		marker_status[get_id_index(marker_id, true)]=1;
	}
}

void MultiMarker::PointCloudCopy(const MultiMarker *m) {
	pointcloud.clear();
	pointcloud = m->pointcloud; // TODO: Is this copy operation ok?
	marker_indices.resize(m->marker_indices.size());
	marker_status.resize(m->marker_status.size());
	copy(m->marker_indices.begin(), m->marker_indices.end(), marker_indices.begin());
	copy(m->marker_status.begin(), m->marker_status.end(), marker_status.begin());
}

void MultiMarker::PointCloudGet(int marker_id, int point,
                                double &x, double &y, double &z) {
  CvPoint3D64f p3d = pointcloud[pointcloud_index(marker_id, point)];
  x = p3d.x;
  y = p3d.y;
  z = p3d.z;
}

bool MultiMarker::IsValidMarker(int marker_id) {
	int idx = get_id_index(marker_id);
	return idx != -1 && marker_status[idx] != 0;
}


double MultiMarker::_GetPose(MarkerIterator &begin, MarkerIterator &end, Camera* cam, Pose& pose, IplImage* image) {
	vector<CvPoint3D64f> world_points;
	vector<PointDouble>  image_points;

	// Reset the marker_status to 1 for all markers in point_cloud
	for (size_t i=0; i<marker_status.size(); i++) {
		if (marker_status[i] > 0) marker_status[i]=1;
	}

	// For every detected marker
	for (MarkerIterator &i = begin.reset(); i != end; ++i)
	{
		const Marker* marker = *i;
		int id = marker->GetId();
		int index = get_id_index(id);
		if (index < 0) continue;

		// But only if we have corresponding points in the pointcloud
		if (marker_status[index] > 0) {
			for(size_t j = 0; j < marker->marker_corners.size(); ++j)
			{
				CvPoint3D64f Xnew = pointcloud[pointcloud_index(id, (int)j)];
				world_points.push_back(Xnew);
				image_points.push_back(marker->marker_corners_img.at(j));
				if (image) cvCircle(image, cvPoint(int(marker->marker_corners_img[j].x), int(marker->marker_corners_img[j].y)), 3, CV_RGB(0,255,0));
			}
			marker_status[index] = 2; // Used for tracking
		}
	}

	if (world_points.size() < 4) return -1;

	double rod[3], tra[3];
	CvMat rot_mat = cvMat(3, 1,CV_64F, rod);
	CvMat tra_mat = cvMat(3, 1,CV_64F, tra);
	double error=0; // TODO: Now we don't calculate any error value
	cam->CalcExteriorOrientation(world_points, image_points, &rot_mat, &tra_mat);
	pose.SetRodriques(&rot_mat);
	pose.SetTranslation(&tra_mat);
	return error;
}


int MultiMarker::_SetTrackMarkers(MarkerDetectorImpl &marker_detector, Camera* cam, Pose& pose, IplImage *image) {
	int count=0;
	marker_detector.TrackMarkersReset();
	for(size_t i = 0; i < marker_indices.size(); ++i) {
		int id = marker_indices[i];
		// If the marker wasn't tracked lets add it to be trackable
		if (marker_status[i] == 1) {
			vector<CvPoint3D64f> pw(4);
			pw[0] = pointcloud[pointcloud_index(id, 0)];
			pw[1] = pointcloud[pointcloud_index(id, 1)];
			pw[2] = pointcloud[pointcloud_index(id, 2)];
			pw[3] = pointcloud[pointcloud_index(id, 3)];
			vector<CvPoint2D64f> pi(4);
			cam->ProjectPoints(pw, &pose, pi);
			PointDouble p[4]; // TODO: This type copying is so silly!!!
			p[0].x = pi[0].x;
			p[0].y = pi[0].y;
			p[1].x = pi[1].x;
			p[1].y = pi[1].y;
			p[2].x = pi[2].x;
			p[2].y = pi[2].y;
			p[3].x = pi[3].x;
			p[3].y = pi[3].y;
			if (image) {
				cvLine(image, cvPoint(int(p[0].x), int(p[0].y)), cvPoint(int(p[1].x), int(p[1].y)), CV_RGB(255,0,0));
				cvLine(image, cvPoint(int(p[1].x), int(p[1].y)), cvPoint(int(p[2].x), int(p[2].y)), CV_RGB(255,0,0));
				cvLine(image, cvPoint(int(p[2].x), int(p[2].y)), cvPoint(int(p[3].x), int(p[3].y)), CV_RGB(255,0,0));
				cvLine(image, cvPoint(int(p[3].x), int(p[3].y)), cvPoint(int(p[0].x), int(p[0].y)), CV_RGB(255,0,0));
			}
			marker_detector.TrackMarkerAdd(id, p);
			count++;
		}
	}
	return count;
}

}

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

#include "Util.h"
#include "FileFormatUtils.h"

using namespace std;

namespace alvar {
using namespace std;

//ttesis

int dot(CvPoint *A, CvPoint *B, CvPoint *C){
    CvPoint AB, BC; 
    AB.x = B->x-A->x;
	AB.y = B->y-A->y;
    BC.x = C->x-B->x;
    BC.y = C->y-B->y;
    int dot = AB.x * BC.x + AB.y * BC.y;
    return dot;
}

int cross(CvPoint *A,CvPoint *B, CvPoint *C){
	CvPoint AB, AC; 
	AB.x = B->x-A->x;
	AB.y = B->y-A->y;
	AC.x = C->x-A->x;
    AC.y = C->y-A->y;
    int cross = AB.x * AC.y - AB.y * AC.x;
    return cross;
}

double distance(CvPoint *A,CvPoint *B){
    double d1 = A->x - B->x;
	double d2 = A->y - B->y;
    return sqrt(d1*d1+d2*d2);
}


double linePointDist(CvPoint *A,CvPoint *B,CvPoint *C, bool isSegment){
	double dist = cross(A,B,C) / distance(A,B);
    if(isSegment){
        int dot1 = dot(A,B,C);
        if(dot1 > 0)return distance(B,C);
        int dot2 = dot(B,A,C);
        if(dot2 > 0)return distance(A,C);
    }
    return abs(dist);
}

double angle(CvPoint *A,CvPoint *B, CvPoint *C,CvPoint *D, int isDirectionDependent){
	double angle;
	double a = B->x  - A->x; 
	double b = B->y  - A->y;
	double c = D->x - C->x;
	double d = D->y - C->y;
	angle = acos( ((a * c) + (b * d) )/(sqrt(a*a + b*b) * sqrt(c*c + d*d)));
	if(isDirectionDependent){
		return angle;
	}else{
		if (angle > CV_PI/2){
			return CV_PI - angle;
		}else
			return angle;
	}	
}

double polyLinePointDist(CvPoint *PointList, int nPnts,CvPoint *C, int *index, int isClosedPolygon){
//	Calculate minimum distance of Point C to Polygon whose points are in list PointList
//	if isClosedPolygon is true polygon is closed (segnment of the first and last point is also checked)
//	index is the index of point A in pointlist, 
//	where A is the starting point of the closest polygon segment
	*index = -1;
	double mindist= -1;
	double dist;
	for( int i=0; i < nPnts-1; i++){
		dist=linePointDist(&PointList[i],&PointList[i+1],C,1);
		if (mindist == -1 || dist<mindist){
			mindist = dist;
			*index = i;
		}
	}
	if(isClosedPolygon){
		dist=linePointDist(&PointList[nPnts-1],&PointList[0],C,1);
		if (dist<mindist){
			mindist = dist;
			*index = nPnts-1;
		}
	}
	return mindist;

}

//ttesis

void FitCVEllipse(const vector<PointDouble> &points, CvBox2D& ellipse_box)
{
	if(points.size() < 8) return;

	CvMat* vector = cvCreateMat(1, int(points.size()), CV_64FC2);
	for(size_t i = 0; i < points.size(); ++i)
	{
		CV_MAT_ELEM(*vector, CvPoint2D64f, 0, i) = (CvPoint2D64f)points[i];
	}
	ellipse_box = cvFitEllipse2(vector);
	cvReleaseMat(&vector);
}

int exp_filt2(vector<double> &v, vector<double> &ret, bool clamp)
{
	double y;
	int n = (int)v.size();
	
	double a = pow(0.01, 8.0/n);//0.8;
	double k = -log(a);
	
	// Forward
	vector<double> yp(n);
	
	y = 0;
	for(int i = 0; i < n; ++i)
		y = a * y + v[i];
	
	y *= 1.0 / (1.0-pow(a,n));

	for(int i = 0; i < n; ++i)
	{
		y = a * y + v[i];
		yp[i] = y;
	}

	// Backward
	vector<double> ym(n);
	
	y = 0;
	for(int i = n-1; i >= 0; --i)
		y = a * y + v[i];
	
	y *= 1.0 / (1.0-pow(a,n));

	for(int i = n-1; i >= 0; --i)
	{
		y = a * y + v[i];
		ym[i] = y;
	}

	// Filter
	ret.resize(n);
	for(int i = 0; i < n; ++i)
	{
		ret[i] = (k/2.0) * (yp[i] + ym[i] - v[i]);
	}
	
	return int(ret.size());
}

int find_zero_crossings(const vector<double>& v, vector<int> &corners, int offs)
{
	int ind = 0;
	int len = (int)v.size();

	int state;
	if(Sign(v.at(0)) == 1)
		state = 1;
	else
		state = 2;
	
	corners.clear();
	for(int i = 0; i < len+offs; ++i)
	{
		if(i<len)
			ind = i;
		else
			ind = i-len;
		
		int s = Sign(v.at(ind));
		if(state == 1 && s == -1)
			state = 2;
		if(state == 2 && s == 1)
		{
			state = 1;
			bool test = true;
			for(unsigned j = 0; j < corners.size(); ++j)
				if(corners.at(j) == ind)
					test = false;
						
			if(test)	
				corners.push_back(ind);
		}
	}
	
	return int(corners.size());
}

void out_matrix(const CvMat *m, const char *name) {
	if (m->cols == 1) {
		std::cout<<name<<" = [";
		for (int j=0; j<m->rows; j++) {
			std::cout<<" "<<cvGet2D(m, j, 0).val[0];
		}
		std::cout<<"]^T"<<std::endl;
	} else if (m->rows == 1) {
		std::cout<<name<<" = [";
		for (int i=0; i<m->cols; i++) {
			std::cout<<" "<<cvGet2D(m, 0, i).val[0];
		}
		std::cout<<"]^T"<<std::endl;
	} else {
		std::cout<<name<<" = ["<<std::endl;
		for (int j=0; j<m->rows; j++) {
			for (int i=0; i<m->cols; i++) {
				std::cout<<" "<<cvGet2D(m, j, i).val[0];
			}
			std::cout<<std::endl;
		}
		std::cout<<"]"<<std::endl;
	}
}

double Limit(double val, double min_val, double max_val) {
	return max(min_val, min(max_val, val));
}

Index::Index(int a) { val.push_back(a); }

Index::Index(int a, int b) {
	val.push_back(a); 
	val.push_back(b); 
}

Index::Index(int a, int b, int c) { 
	val.push_back(a); 
	val.push_back(b); 
	val.push_back(c); 
}

bool Index::operator<(const Index &index) const {
	int comp=0;
	size_t d=0;
	// Go through the dimensions (last being the most significant)
	while ((d < val.size()) || (d < index.val.size())) {
		int v0 = (d < val.size() ? val[d] : 0);
		int v1 = (d < index.val.size() ? index.val[d] : 0);
		if (v0<v1) comp=-1;
		else if (v1<v0) comp=1;
		d++;
	}
	if (comp == -1) return true;
	return false;
}

int Histogram::DimIndex(int dim, double val) {
	int binsize = (dim < int(dim_binsize.size()) ? dim_binsize[dim] : 1);
	if (val >= 0) return int(val+(binsize/2))/binsize;
	return int(val-(binsize/2))/binsize;
}
double Histogram::DimVal(int dim, int index) {
	int binsize = (dim < int(dim_binsize.size()) ? dim_binsize[dim] : 1);
	return (index * binsize);
}
void Histogram::AddDimension(int binsize) {
	dim_binsize.push_back(binsize);
}
void Histogram::Clear() {
	bins.clear();
}
void Histogram::Inc(double dim0, double dim1, double dim2) {
	Index index(DimIndex(0,dim0), DimIndex(1,dim1), DimIndex(2,dim2));
	if (bins.find(index) != bins.end()) {
		bins[index]++;
	} else {
		bins[index] = 1;
	}
}
int Histogram::GetMax(double *dim0, double *dim1, double *dim2) {
	map<Index, int>::const_iterator iter, max_iter;
	int max=0;
	for (max_iter = iter = bins.begin(); iter != bins.end(); iter++) {
		if (iter->second > max) {
			max = iter->second;
			max_iter = iter;
		}
	}
	if (max > 0) {
		*dim0 = DimVal(0, max_iter->first.val[0]);
		if (dim1) *dim1 = DimVal(1, max_iter->first.val[1]);
		if (dim2) *dim2 = DimVal(2, max_iter->first.val[2]);
	}
	return max;
}
void HistogramSubpixel::Clear() {
	bins.clear();
	acc_dim0.clear();
	acc_dim1.clear();
	acc_dim2.clear();
}
void HistogramSubpixel::Inc(double dim0, double dim1, double dim2) {
	Index index(DimIndex(0,dim0), DimIndex(1,dim1), DimIndex(2,dim2));
	if (bins.find(index) != bins.end()) {
		bins[index]++;
		acc_dim0[index] += dim0;
		acc_dim1[index] += dim1;
		acc_dim2[index] += dim2;
	} else {
		bins[index] = 1;
		acc_dim0[index] = dim0;
		acc_dim1[index] = dim1;
		acc_dim2[index] = dim2;
	}
}
int HistogramSubpixel::GetMax(double *dim0, double *dim1, double *dim2) {
	map<Index, int>::const_iterator iter;
	int max=0;
	int divider=0;
	for (iter = bins.begin(); iter != bins.end(); iter++) {
		if (iter->second > max) {
			divider = max = iter->second;
			*dim0 = acc_dim0[iter->first];
			if (dim1) *dim1 = acc_dim1[iter->first];
			if (dim2) *dim2 = acc_dim2[iter->first];
		} else if (iter->second == max) {
			divider += iter->second;
			*dim0 += acc_dim0[iter->first];
			if (dim1) *dim1 += acc_dim1[iter->first];
			if (dim2) *dim2 += acc_dim2[iter->first];
		}
	}
	if (max > 0) {
		*dim0 /= divider;
		if (dim1) *dim1 /= divider;
		if (dim2) *dim2 /= divider;
	}
	return max;
}

struct SerializationFormatterXml {
	TiXmlDocument document;
	TiXmlElement *xml_current;
	SerializationFormatterXml() : xml_current(0) {}
};

bool Serialization::Output() {
	SerializationFormatterXml *xml = (SerializationFormatterXml *)formatter_handle;
	if (filename.size() > 0) {
		//xml->document.LinkEndChild(new TiXmlDeclaration("1.0", "UTF-8", "no"));
		xml->document.InsertBeforeChild	(xml->document.RootElement(), TiXmlDeclaration("1.0", "UTF-8", "no"));
		xml->document.SaveFile(filename.c_str());
	} else {
		const TiXmlNode *node = (xml->xml_current ? xml->xml_current : xml->document.RootElement());
		std::basic_ostream<char> *os = dynamic_cast<std::basic_ostream<char> *>(stream);
		(*os)<<(*node);
		//(*stream)<<(*node);
	}
	return true;
}

bool Serialization::Input() {
	SerializationFormatterXml *xml = (SerializationFormatterXml *)formatter_handle;
	if (filename.size() > 0) {
		xml->document.LoadFile(filename.c_str());
	} else {
		// TODO: How this should be handled with nested classes?
		TiXmlNode *node = (xml->xml_current ? xml->xml_current : xml->document.RootElement());
		if (node == 0) {
			node = (TiXmlElement*)xml->document.LinkEndChild(new TiXmlElement("root"));
		}
		std::basic_istream<char> *is = dynamic_cast<std::basic_istream<char> *>(stream);
		(*is)>>(*node);
		//(*stream)>>(*node);
	}
	return true;
}

bool Serialization::Descend(const char *id) {
	SerializationFormatterXml *xml = (SerializationFormatterXml *)formatter_handle;
	if (input) {
		if (xml->xml_current == 0) {
			xml->xml_current = xml->document.RootElement();
			if (!xml->xml_current || (strcmp(xml->xml_current->Value(), id) != 0)) {
				return false;
			}
		} else {
			xml->xml_current = (TiXmlElement*)xml->xml_current->FirstChild (id);
			if (xml->xml_current == NULL) return false;
		}
	} else {
		if (xml->xml_current == 0) {
			xml->xml_current = (TiXmlElement*)xml->document.LinkEndChild(new TiXmlElement(id));
		} else {
			xml->xml_current = (TiXmlElement*)xml->xml_current->LinkEndChild(new TiXmlElement(id));
		}
	}
	return true;
}
bool Serialization::Ascend() {
	SerializationFormatterXml *xml = (SerializationFormatterXml *)formatter_handle;
	xml->xml_current = (TiXmlElement*)xml->xml_current->Parent();
	return true;
}

Serialization::Serialization(std::string _filename) {
	SerializationFormatterXml *xml = new SerializationFormatterXml();
	formatter_handle = xml;
	filename = _filename;
	input = false; // by default output
}

Serialization::Serialization(std::basic_iostream<char> &_stream)
{
	SerializationFormatterXml *xml = new SerializationFormatterXml();
	formatter_handle = xml;
	stream = &_stream;
}

Serialization::Serialization(std::basic_istream<char> &_stream) {
	SerializationFormatterXml *xml = new SerializationFormatterXml();
	formatter_handle = xml;
	stream = &_stream;
}

Serialization::Serialization(std::basic_ostream<char> &_stream) {
	SerializationFormatterXml *xml = new SerializationFormatterXml();
	formatter_handle = xml;
	stream = &_stream;
}

Serialization::~Serialization() {
	SerializationFormatterXml *xml = (SerializationFormatterXml *)formatter_handle;
	delete xml;
}

bool Serialization::Serialize(int &data, const std::string &name) {
	SerializationFormatterXml *xml = (SerializationFormatterXml *)formatter_handle;
	if (!xml || !xml->xml_current) return false;
	bool ret = true;
	if (input) ret = (xml->xml_current->QueryIntAttribute(name, &data) == TIXML_SUCCESS);
	else xml->xml_current->SetAttribute(name, data);
	return ret;
}

bool Serialization::Serialize(unsigned short &data, const std::string &name) {
	int i = data;
	bool ret = Serialize(i, name);
	data = i;
	return ret;
}

bool Serialization::Serialize(unsigned long &data, const std::string &name) {
	// TODO: Possible overflow here...
	int i = data;
	bool ret = Serialize(i, name);
	data = i;
	return ret;
}

bool Serialization::Serialize(double &data, const std::string &name) {
	SerializationFormatterXml *xml = (SerializationFormatterXml *)formatter_handle;
	bool ret = true;
	if (input) ret = (xml->xml_current->QueryDoubleAttribute(name, &data) == TIXML_SUCCESS);
	else xml->xml_current->SetDoubleAttribute(name.c_str(), data);
	return ret;
}

bool Serialization::Serialize(std::string &data, const std::string &name) {
	SerializationFormatterXml *xml = (SerializationFormatterXml *)formatter_handle;
	bool ret = true;
	if (input) {
		const char *tmp = xml->xml_current->Attribute(name.c_str());
		if (tmp == NULL) ret=false;
		else data = tmp;
	}
	else xml->xml_current->SetAttribute(name.c_str(), data.c_str());
	return ret;
}

bool Serialization::Serialize(CvMat &data, const std::string &name) {
	SerializationFormatterXml *xml = (SerializationFormatterXml *)formatter_handle;
	bool ret = true;
	if (input) {
		TiXmlElement *xml_matrix = (TiXmlElement*)xml->xml_current->FirstChild(name);
		if (xml_matrix == NULL) return false;
		if (!FileFormatUtils::parseXMLMatrix(xml_matrix, &data)) return false;
	}
	else {
		xml->xml_current->LinkEndChild(FileFormatUtils::createXMLMatrix(name.c_str(), &data));
	}
	return ret;
}

} // namespace alvar

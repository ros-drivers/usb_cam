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

#ifndef UTIL_H
#define UTIL_H

/**
 * \file Util.h
 *
 * \brief This file implements generic utility functions and a serialization
 * interface.
 */

#include "Alvar.h"
#include "AlvarException.h"
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cxcore.h>
#include <cv.h>
#include <cmath> //for abs
#include <map>

namespace alvar {

const double PI = 3.14159265;

/**
* \brief Returns the sign of a number.
*/
template<class C> inline
int ALVAR_EXPORT Sign(const C& v)
{
	return (v<0?-1:1);
}

/**
* \brief Converts an angle from radians to degrees.
*/
template<class C> inline
double ALVAR_EXPORT Rad2Deg(const C& v)
{
	return v*(180/PI);
}

/**
* \brief Converts an angle from degrees to radians.
*/
template<class C> inline
double ALVAR_EXPORT Deg2Rad(const C& v)
{
	return v*(PI/180);
}

/**
 * \brief Simple \e Point class meant to be inherited from OpenCV point-classes. For example: Point<CvPoint2D64f> p
 */
template<class C, class D = int> 
struct ALVAR_EXPORT Point : public C
{
	/**
	 * \brief Additional value can be related to the point.
	 */
	D val;

	Point(int vx=0, int vy=0)
	{
		C::x = vx;
		C::y = vy;
	}
	Point(double vx, double vy)
	{
		C::x = vx;
		C::y = vy;
	}
};

/** 
  *  \brief The default integer point type.
*/
typedef ALVAR_EXPORT Point<CvPoint> PointInt;

/**
  *  \brief The default double point type.
*/
typedef ALVAR_EXPORT Point<CvPoint2D64f> PointDouble;

/** \brief Returns the squared distance of two points. 
  * \param p1	First point.
  * \param p2	Second point.
  * \return Squared distance.
*/
template<class PointType>
double PointSquaredDistance(PointType p1, PointType p2) {
	return ((p1.x-p2.x)*(p1.x-p2.x)) +
		((p1.y-p2.y)*(p1.y-p2.y));
}


//ttesis start


/** 
  * \brief  Computes dot product AB.BC
  * \param  A,B and C	points defining lines (line segments) AB and BC
 */
int ALVAR_EXPORT dot(CvPoint *A, CvPoint *B, CvPoint *C);

/** 
  * \brief  Computes the cross product AB x AC
  * \param  A,B and C points defining lines (line segments) AB and AC
  * \param 
 */
int ALVAR_EXPORT cross(CvPoint *A,CvPoint *B, CvPoint *C);


/** 
  * \brief  Compute the distance from A to B
  * \param  A and B		points
 */
double ALVAR_EXPORT distance(CvPoint *A,CvPoint *B);

/** 
  * \brief  Computes the distance from point C to line (segment) AB.
  * \param  isSegment	If isSegment is true, AB is a segment, not a line.
  * \param  C	point
  * \param  A abd B	 points defining line (segment) AB
 */
double ALVAR_EXPORT linePointDist(CvPoint *A,CvPoint *B,CvPoint *C, bool isSegment);


/** 
  * \brief  Computes the angle between lines AB and CD
  * \param  isDirectionDependent	If isDirectionDependent = 1, angle depends on the order of the points. Otherwise returns smaller angle.
  * \param  A start point of first line
  * \param  B end point of first line
  * \param  C start point of second line
  * \param  D end point of second line
 */
double ALVAR_EXPORT angle(CvPoint *A,CvPoint *B, CvPoint *C,CvPoint *D, int isDirectionDependent);


/** 
  * \brief  Calculates minimum distance from Point C to Polygon whose points are in list PointList
  * \brief  Returns distance
  * \param  index	index of point A in pointlist, where A is the starting point of the closest polygon segment
  * \param  isClosedPolygon is true if polygon is closed (segment of the first and last point is also checked)
 */
double ALVAR_EXPORT polyLinePointDist(CvPoint *PointList, int nPnts,CvPoint *C, int *index, int isClosedPolygon);

//ttesis end


/** 
  * \brief Uses OpenCV routine to fit ellipse to a vector of points. 
  * \param points		Vector of points on the ellipse edge.
  * \param ellipse_box	OpenCV struct for the fitted ellipse.
 */
void ALVAR_EXPORT FitCVEllipse(const std::vector<PointDouble> &points, CvBox2D& ellipse_box);

int ALVAR_EXPORT exp_filt2(std::vector<double> &v,std:: vector<double> &ret, bool clamp);

/** 
  * \brief Calculates the difference between the consecutive vector elements.
  * \param v	Source elements.
  * \param ret	The difference vector. This is cleared and then resized.
  * \return		The number of elements.
  */
template<class C> inline
int ALVAR_EXPORT diff(const std::vector<C> &v, std::vector<C> &ret)
{	
	ret.clear();
	if (v.size() == 1) {
		ret.push_back(0);
	} else if (v.size() > 1) {
		ret.push_back(v.at(1)-v.at(0));
		for(size_t i = 1; i < v.size(); ++i)
		{
			ret.push_back(v.at(i)-v.at(i-1));
		}
	}
	return int(ret.size());
}

/** 
  * \brief Finds zero crossings of given vector elements (sequence).
  * \param v		Sequence of numbers from where the zero crossings are found.
  * \param corners	Resulting index list of zero crossings.
  * \param offs		
  * \return			Number of zero crossings found.
  */
int ALVAR_EXPORT find_zero_crossings(const std::vector<double>& v, std::vector<int> &corners, int offs = 20);

/** 
  * \brief Output OpenCV matrix for debug purposes.
  */
void ALVAR_EXPORT out_matrix(const CvMat *m, const char *name);

/** 
  * \brief Limits a number to between two values. 
  * \param val		Input value.
  * \param min_val	Minimum value for the result.
  * \param max_val	Maximum value for the result.
  * \return			Resulting value that is between \e min_val and \e max_val.
*/
double ALVAR_EXPORT Limit(double val, double min_val, double max_val);

/** 
 * \brief Class for N-dimensional index to be used e.g. with STL maps
 *
 * The idea is that if you want to sort N-dimensional pointers (e.g.
 * when they are stored in STL maps) it is enough to have the 'operator<'
 * working, instead of needing to calculate something like i=z*x_res*y_res + y*x_res + x;
 */
struct ALVAR_EXPORT Index {
	/** \brief The indices for each dimension are stored in \e val (last being the most significant) */
	std::vector<int> val;
	/** \brief Constructor for 1D index */
	Index(int a);
	/** \brief Constructor for 2D index */
	Index(int a, int b);
	/** \brief Constructor for 3D index */
	Index(int a, int b, int c);
	/** \brief Operator used for sorting the multidimensional indices (last dimension being the most significant) */
	bool operator<(const Index &index) const;
};

/** 
 * \brief Class for N-dimensional Histograms
 */
class ALVAR_EXPORT Histogram {
protected:
	std::map<Index, int> bins;
	std::vector<int> dim_binsize;
	int DimIndex(int dim, double val);
	double DimVal(int dim, int index);
public:
	/** \brief Add dimension with a binsize 
	 */
	void AddDimension(int binsize);
	/** \brief Clear the histogram */
	void Clear();
	/** \brief Increase the histogram for given dimensions */
	void Inc(double dim0, double dim1=0, double dim2=0);
	/** \brief Get the maximum from the histogram 
	 *  This returns the value in the middle of the 'bin' instead of bin-number
	 */
	int GetMax(double *dim0, double *dim1=0, double *dim2=0);
};

/** 
 * \brief N-dimensional Histograms calculating also the subpixel average for max bin
 */
class ALVAR_EXPORT HistogramSubpixel : public Histogram {
protected:
	std::map<Index, double> acc_dim0;
	std::map<Index, double> acc_dim1;
	std::map<Index, double> acc_dim2;
public:
	/** \brief Clear the histogram */
	void Clear();
	/** \brief Increase the histogram for given dimensions */
	void Inc(double dim0, double dim1=0, double dim2=0);
	/** \brief Get the maximum from the histogram 
	 *  This finds the maximum bin(s) and averages the original
	 *  values contained there to achieve subpixel accuracy.
	 */
	int GetMax(double *dim0, double *dim1=0, double *dim2=0);
};

#if (_MSC_VER >= 1400)
	inline void STRCPY(char *to, rsize_t size, const char *src) {
		strcpy_s(to,size,src);
	}
#else
	inline void STRCPY(char *to, size_t size, const char *src) {
		strncpy(to,src,size-1);
	}
#endif

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

/** \brief Class for serializing class content to/from file or std::iostream
 *
 * The class is mainly meant to serialize classes that implement
 * two required methods \e SerializeId and \e Serialize . For example
 * alvar::Camera implements the following to make it serializable:
 * \code
 * const char *SerializeId { return "camera"; };
 * bool Serialize(Serialization *ser) {
 *	if (!ser->Serialize(calib_x_res, "width")) return false;
 *	if (!ser->Serialize(calib_y_res, "height")) return false;
 *	if (!ser->Serialize(calib_K, "intrinsic_matrix")) return false;
 *	if (!ser->Serialize(calib_D, "distortion")) return false;
 *	return true;
 * }
 * \endcode
 * In your classes \e Serialize -method you can use the overloaded 
 * \e Serialize method of the \e Serialization class to serialize
 * data or data arrays. In addition you can use \e SerializeClass
 * to serialize inner serializable classes.
 *
 * After the class is serializable i.e. it implements the above two
 * methods you can serialize it as follows (some examples):
 * \code
 * alvar::Camera cam;
 * cam.SetCalib("calib.xml", 320, 240);
 * Serialization sero(std::cout);
 * sero<<cam;
 * \endcode
 * \code
 * std::stringstream ss;
 * Serialization sero(ss);
 * sero<<cam;
 * std::cout<<ss.str()<<std::endl;
 * // ...
 * Serialization seri(ss);
 * seri>>cam;
 * \endcode
 *
 * See the constructor \e Serialization::Serialization documentation for 
 * further use examples.
 */
class ALVAR_EXPORT Serialization {
protected:
	bool input;
	std::string filename;
	//std::iostream *stream;
	std::ios *stream;
	void *formatter_handle;
	bool Output();
	bool Input();
	bool Descend(const char *id);
	bool Ascend();
public:
	/** \brief Constructor for serializing to/from specified filename 
	 *
	 * \code
	 * Serialization sero("test1.xml");
	 * sero<<cam;
	 * \endcode
	 * \code
	 * Serialization seri("test1.xml");
	 * seri>>cam;
	 * \endcode
	 *
	 * Note that this is not identical to:
	 * \code
	 * ofstream ofs("test1.xml");
	 * Serialization sero(ofs);
	 * sero<<cam;
	 * \endcode
	 * \code
	 * ifstream ifs("test1.xml");
	 * Serialization seri(ifs);
	 * sero>>cam;
	 * \endcode
	 * 
	 * There are differences with these approaches. When using the constructor 
	 * with 'filename', we use the tinyxml Save and Load methods, while with
	 * iostream we use tinyxml operators for << and >> . The prior approach
	 * uses properly indented xml-files with XML declaration <?...?>. In the 
	 * latter approach the indentations and the XML declaration are left out.
	 * The XML declaration <?...?> is left out because for some reason tinyxml 
	 * doesn't parse it correctly when using operator>> .
	 */
	Serialization(std::string _filename);
	/** \brief Constructor for serializing any iostream (e.g. std::stringstream) */
	Serialization(std::basic_iostream<char> &_stream);
	/** \brief Constructor for serializing any istream (e.g. std::cin) */
	Serialization(std::basic_istream<char> &_stream);
	/** \brief Constructor for serializing any ostream (e.g. std::cout) */
	Serialization(std::basic_ostream<char> &_stream);
	/** \brief Destructor */
	~Serialization();
	/** \brief Operator for outputting a serializable class into the defined filename or std::iostream */
	template <class C>
	Serialization& operator<<(C &serializable) {
		input=false;
		if (!SerializeClass(serializable) || !Output()) {
			throw(AlvarException("Serialization failure"));
		}
		return *this;
	}
	/** \brief Operator for reading a serializable class from the defined filename or std::iostream */
	template <class C>
	Serialization& operator>>(C &serializable) {
		input=true;
		if (!Input() || !SerializeClass(serializable)) {
			throw(AlvarException("Serialization failure"));
		}
		return *this;
	}
	/** \brief Method for serializing a serializable class. Used by operators << and >> . 
	 *
	 * Note, in the future this should be usable also from your serializable class
	 * for adding nested serializable classes.
	 */
	template <class C>
	bool SerializeClass(C &serializable) {
		std::string s = serializable.SerializeId();
		if (!Descend(s.c_str()) || !serializable.Serialize(this) || !Ascend()) {
			return false;
		}
		return true;
	}
	/** \brief Method for serializing 'int' data element. Used from your serializable class. */
	bool Serialize(int &data, const std::string &name);
	/** \brief Method for serializing 'int' data element. Used from your serializable class. */
	bool Serialize(unsigned short &data, const std::string &name);
	/** \brief Method for serializing 'int' data element. Used from your serializable class. */
	bool Serialize(unsigned long &data, const std::string &name);
	/** \brief Method for serializing 'double' data element. Used from your serializable class. */
	bool Serialize(double &data, const std::string &name);
	/** \brief Method for serializing 'std::string' data element. Used from your serializable class. */
	bool Serialize(std::string &data, const std::string &name);
	/** \brief Method for serializing 'CvMat' data element. Used from your serializable class. */
	bool Serialize(CvMat &data, const std::string &name);
	/** \brief Method for checking if we are inputting or outputting. Can be used from your serializable class. */
	bool IsInput() { return input; }
};

} // namespace alvar

#endif

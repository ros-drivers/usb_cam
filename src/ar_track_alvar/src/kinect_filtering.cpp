
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */



//#include <ros/ros.h>
#include <Eigen/Core>
#include <ar_track_alvar/kinect_filtering.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>


namespace ar_track_alvar
{

  namespace gm=geometry_msgs;

  using std::vector;
  using std::cerr;
  using std::endl;
  using std::ostream;

  // Distance threshold for plane fitting: how far are points
  // allowed to be off the plane?
  const double distance_threshold_ = 0.005;

  PlaneFitResult fitPlane (ARCloud::ConstPtr cloud)
  {
    PlaneFitResult res;
    pcl::PointIndices::Ptr inliers=boost::make_shared<pcl::PointIndices>();

    pcl::SACSegmentation<ARPoint> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold_);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, res.coeffs);

    pcl::ExtractIndices<ARPoint> extracter;
    extracter.setInputCloud(cloud);
    extracter.setIndices(inliers);
    extracter.setNegative(false);
    extracter.filter(*res.inliers);
  
    return res;
  }

  ARCloud::Ptr filterCloud (const ARCloud& cloud, const vector<cv::Point>& pixels)
  {
    ARCloud::Ptr out(new ARCloud());
    //ROS_INFO("  Filtering %zu pixels", pixels.size());
    //for (const cv::Point& p : pixels)
    for(size_t i=0; i<pixels.size(); i++)
      {
	const cv::Point& p = pixels[i];
	const ARPoint& pt = cloud(p.x, p.y);
	if (isnan(pt.x) || isnan(pt.y) || isnan(pt.z)){
	  //ROS_INFO("    Skipping (%.4f, %.4f, %.4f)", pt.x, pt.y, pt.z);
	}
	else
	  out->points.push_back(pt);
      }
    return out;
  }

  gm::Point centroid (const ARCloud& points)
  {
    gm::Point sum;
    sum.x = 0;
    sum.y = 0;
    sum.z = 0;
    //for (const Point& p : points)
    for(size_t i=0; i<points.size(); i++)
      {
	sum.x += points[i].x;
	sum.y += points[i].y;
	sum.z += points[i].z;
      }
  
    gm::Point center;
    const size_t n = points.size();
    center.x = sum.x/n;
    center.y = sum.y/n;
    center.z = sum.z/n;
    return center;
  }

  // Helper function to construct a geometry_msgs::Quaternion
  inline
  gm::Quaternion makeQuaternion (double x, double y, double z, double w)
  {
    gm::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
  }

  // Extract and normalize plane coefficients
  int getCoeffs (const pcl::ModelCoefficients& coeffs, double* a, double* b,
		 double* c, double* d)
  {
    if(coeffs.values.size() != 4)
      return -1;
    const double s = coeffs.values[0]*coeffs.values[0] +
      coeffs.values[1]*coeffs.values[1] + coeffs.values[2]*coeffs.values[2];
    if(fabs(s) < 1e-6)
      return -1;
    *a = coeffs.values[0]/s;
    *b = coeffs.values[1]/s;
    *c = coeffs.values[2]/s;
    *d = coeffs.values[3]/s;
    return 0;
  }

  // Project point onto plane
  tf::Vector3 project (const ARPoint& p, const double a, const double b,
		     const double c, const double d)
  {
    const double t = a*p.x + b*p.y + c*p.z + d;
    return tf::Vector3(p.x-t*a, p.y-t*b, p.z-t*c);
  }

  ostream& operator<< (ostream& str, const tf::Matrix3x3& m)
  {
    str << "[" << m[0][0] << ", " << m[0][1] << ", " << m[0][2] << "; "
	<< m[1][0] << ", " << m[1][1] << ", " << m[1][2] << "; "
	<< m[2][0] << ", " << m[2][1] << ", " << m[2][2] << "]";
    return str;
  }

  ostream& operator<< (ostream& str, const tf::Quaternion& q)
  {
    str << "[(" << q.x() << ", " << q.y() << ", " << q.z() <<
      "), " << q.w() << "]";
    return str;
  }

  ostream& operator<< (ostream& str, const tf::Vector3& v)
  {
    str << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    return str;
  }

  int extractFrame (const pcl::ModelCoefficients& coeffs,
		    const ARPoint& p1, const ARPoint& p2,
		    const ARPoint& p3, const ARPoint& p4,
            tf::Matrix3x3 &retmat)
  {
    // Get plane coeffs and project points onto the plane
    double a=0, b=0, c=0, d=0;
    if(getCoeffs(coeffs, &a, &b, &c, &d) < 0)
      return -1;
  
    const tf::Vector3 q1 = project(p1, a, b, c, d);
    const tf::Vector3 q2 = project(p2, a, b, c, d);
    const tf::Vector3 q3 = project(p3, a, b, c, d);
    const tf::Vector3 q4 = project(p4, a, b, c, d);
  
    // Make sure points aren't the same so things are well-defined
    if((q2-q1).length() < 1e-3)
      return -1;
  
    // (inverse) matrix with the given properties
    const tf::Vector3 v = (q2-q1).normalized();
    const tf::Vector3 n(a, b, c);
    const tf::Vector3 w = -v.cross(n);
    tf::Matrix3x3 m(v[0], v[1], v[2], w[0], w[1], w[2], n[0], n[1], n[2]);
  
    // Possibly flip things based on third point
    const tf::Vector3 diff = (q4-q3).normalized();
    //ROS_INFO_STREAM("w = " << w << " and d = " << diff);
    if (w.dot(diff)<0)
      {
	//ROS_INFO_STREAM("Flipping normal based on p3.  Current value: " << m);
	m[1] = -m[1];
	m[2] = -m[2];
	//ROS_INFO_STREAM("New value: " << m);
      }

    // Invert and return
    retmat = m.inverse();
    //cerr << "Frame is " << retmat << endl;
    return 0;
  }


  int getQuaternion (const tf::Matrix3x3& m, tf::Quaternion &retQ)
  {
    if(m.determinant() <= 0)
      return -1;
    
    //tfScalar y=0, p=0, r=0;
    //m.getEulerZYX(y, p, r);
    //retQ.setEulerZYX(y, p, r);

    //Use Eigen for this part instead, because the ROS version of bullet appears to have a bug
    Eigen::Matrix3f eig_m;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            eig_m(i,j) = m[i][j];
        }
    }
    Eigen::Quaternion<float> eig_quat(eig_m);
    
    // Translate back to bullet
    tfScalar ex = eig_quat.x();
    tfScalar ey = eig_quat.y();
    tfScalar ez = eig_quat.z();
    tfScalar ew = eig_quat.w();
    tf::Quaternion quat(ex,ey,ez,ew);
    retQ = quat.normalized();
    
    return 0;
  }


  int extractOrientation (const pcl::ModelCoefficients& coeffs,
			  const ARPoint& p1, const ARPoint& p2,
			  const ARPoint& p3, const ARPoint& p4,
			  gm::Quaternion &retQ)
  {
    tf::Matrix3x3 m;
    if(extractFrame(coeffs, p1, p2, p3, p4, m) < 0)
      return -1;
    tf::Quaternion q;
    if(getQuaternion(m,q) < 0)
      return -1;
    retQ.x = q.x();
    retQ.y = q.y();
    retQ.z = q.z();
    retQ.w = q.w();
    return 0;
  }

} // namespace

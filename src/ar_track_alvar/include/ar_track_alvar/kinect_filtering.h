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

/**
 * \file 
 * 
 * Library for depth based filtering
 *
 * \author Bhaskara Marthi
 */

#ifndef AR_TRACK_ALVAR_KINECT_FILTERING_H
#define AR_TRACK_ALVAR_KINECT_FILTERING_H

#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>

#include <tf/LinearMath/Matrix3x3.h>

namespace ar_track_alvar
{

typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;

// Result of plane fit: inliers and the plane equation
struct PlaneFitResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
  PlaneFitResult () : inliers(boost::make_shared<ARCloud>()) {}
  ARCloud::Ptr inliers;
  pcl::ModelCoefficients coeffs;
};

// Select out a subset of a cloud corresponding to a set of pixel coordinates
ARCloud::Ptr filterCloud (const ARCloud& cloud,
                          const std::vector<cv::Point>& pixels);

// Wrapper for PCL plane fitting
PlaneFitResult fitPlane (ARCloud::ConstPtr cloud);

// Given the coefficients of a plane, and two points p1 and p2, we produce a 
// quaternion q that sends p2'-p1' to (1,0,0) and n to (0,0,1), where p1' and
// p2' are the projections of p1 and p2 onto the plane and n is the normal. 
// There's a sign ambiguity here, which is resolved by requiring that the
// difference p4'-p3' ends up with a positive y coordinate
int
extractOrientation (const pcl::ModelCoefficients& coeffs,
                    const ARPoint& p1, const ARPoint& p2,
                    const ARPoint& p3, const ARPoint& p4,
                    geometry_msgs::Quaternion &retQ);

// Like extractOrientation except return value is a btMatrix3x3
int
extractFrame (const pcl::ModelCoefficients& coeffs,
              const ARPoint& p1, const ARPoint& p2,
              const ARPoint& p3, const ARPoint& p4,
              tf::Matrix3x3 &retmat);


// Return the centroid (mean) of a point cloud
geometry_msgs::Point centroid (const ARCloud& points);
} // namespace

#endif // include guard

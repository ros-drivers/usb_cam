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
 * Test node for kinect filtering
 *
 * \author Bhaskara Marthi
 */

#include <ar_track_alvar/kinect_filtering.h>
#include <fstream>

namespace a=ar_track_alvar;
namespace gm=geometry_msgs;

using std::cerr;
using std::ifstream;

// Random float between a and b
float randFloat (float a, float b)
{
  const float u = static_cast<float>(rand())/RAND_MAX;
  return a + u*(b-a);
}

// Generate points in a square in space of form p+av+bw where 
// a and b range from 0 to 1
a::ARCloud::Ptr generateCloud(const double px, const double py, const double pz,
                              const double vx, const double vy, const double vz,
                              const double wx, const double wy, const double wz)
{
  const double INC=0.1;
  const double NOISE=0.01;

  a::ARCloud::Ptr cloud(boost::make_shared<a::ARCloud>());
  for (double u=0; u<1+INC/2; u+=INC)
  {
    for (double v=0; v<1+INC/2; v+=INC)
    {
      a::ARPoint p;
      p.x = px+u*vx+v*wx+randFloat(-NOISE, NOISE);
      p.y = py+u*vy+v*wy+randFloat(-NOISE, NOISE);
      p.z = pz+u*vz+v*wz+randFloat(-NOISE, NOISE);
      cloud->points.push_back(p);
    }
  }
  return cloud;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "test_points");
  ifstream f("points");
  a::ARCloud::Ptr cloud(new a::ARCloud());
  while (!f.eof())
  {
    a::ARPoint pt;
    f >> pt.x >> pt.y >> pt.z;
    cloud->points.push_back(pt);
  }
  ROS_INFO("Cloud has %zu points such as (%.2f, %.2f, %.2f)",
           cloud->points.size(), cloud->points[0].x, cloud->points[0].y,
           cloud->points[0].z);
  a::ARPoint p1, p2, p3;
  p1.x = 0.1888;
  p1.y = 0.1240;
  p1.z = 0.8620;
  p2.x = 0.0372;
  p2.y = 0.1181;
  p2.z = 0.8670;
  p3.x = 42;
  p3.y = 24;
  p3.z = 88;

  a::PlaneFitResult res = a::fitPlane(cloud);
  ROS_INFO("Plane equation is %.3fx + %.3fy + %.3fz + %.3f = 0",
           res.coeffs.values[0], res.coeffs.values[1], res.coeffs.values[2],
           res.coeffs.values[3]);
  
  gm::Quaternion q = a::extractOrientation(res.coeffs, p1, p2, p3, p1);
  ROS_INFO_STREAM("Orientation is " << q);
  return 0;
}

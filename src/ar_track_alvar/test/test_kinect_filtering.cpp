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

namespace a=ar_track_alvar;
namespace gm=geometry_msgs;

using std::cerr;

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
  if (argc != 12)
  {
    cerr << "Usage: " << argv[0] << " PX PY PZ VX VY VZ WX WY WZ I1 I2 I3\n";
    return 1;
  }
  
  const double px = atof(argv[1]);
  const double py = atof(argv[2]);
  const double pz = atof(argv[3]);
  const double vx = atof(argv[4]);
  const double vy = atof(argv[5]);
  const double vz = atof(argv[6]);
  const double wx = atof(argv[7]);
  const double wy = atof(argv[8]);
  const double wz = atof(argv[9]);

  a::ARCloud::ConstPtr cloud =
    generateCloud(px, py, pz, vx, vy, vz, wx, wy, wz);
  const size_t n = cloud->size();
  ROS_INFO("Generated cloud with %zu points such as (%.4f, %.4f, %.4f)"
           " and (%.4f, %.4f, %.4f)", n, (*cloud)[0].x, (*cloud)[0].y,
           (*cloud)[0].z, (*cloud)[n-1].x, (*cloud)[n-1].y, (*cloud)[n-1].z);
  
  const size_t i1 = atoi(argv[10]);
  const size_t i2 = atoi(argv[11]);
  const size_t i3 = atoi(argv[12]);
  a::ARPoint p1 = (*cloud)[i1];
  a::ARPoint p2 = (*cloud)[i2];
  a::ARPoint p3 = (*cloud)[i3];
  ROS_INFO("Points are (%.4f, %.4f, %.4f) and (%.4f, %.4f, %.4f)",
           p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);

  a::PlaneFitResult res = a::fitPlane(cloud);
  ROS_INFO("Plane equation is %.3fx + %.3fy + %.3fz + %.3f = 0",
           res.coeffs.values[0], res.coeffs.values[1], res.coeffs.values[2],
           res.coeffs.values[3]);
  
  gm::Quaternion q = a::extractOrientation(res.coeffs, p1, p2, p3, p1);
  ROS_INFO_STREAM("Orientation is " << q);

  
  return 0;
}

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
 * N-dimensional median filter for marker poses
 *
 * \author Scott Niekum
 */

#include <ar_track_alvar/medianFilter.h>

namespace ar_track_alvar
{
  using namespace alvar;

  MedianFilter::MedianFilter(int n){
     median_n = n;
     median_ind = 0;
     median_init = false;
     median_poses = new Pose[median_n];
  }

  void MedianFilter::addPose(const Pose &new_pose){
      median_poses[median_ind] = new_pose;
      median_ind = (median_ind+1) % median_n;
  }

  void MedianFilter::getMedian(Pose &ret_pose){
    if(!median_init){
      if(median_ind == median_n-1)
	median_init = true;
      ret_pose = median_poses[median_ind-1];
    }

    else{
      double min_dist = 0;
      int min_ind = 0;
      for(int i=0; i<median_n; i++){
	double total_dist = 0;
	for(int j=0; j<median_n; j++){
	  total_dist += pow(median_poses[i].translation[0] - median_poses[j].translation[0], 2);
	  total_dist += pow(median_poses[i].translation[1] - median_poses[j].translation[1], 2);
	  total_dist += pow(median_poses[i].translation[2] - median_poses[j].translation[2], 2);
	  total_dist += pow(median_poses[i].quaternion[0] - median_poses[j].quaternion[0], 2);
	  total_dist += pow(median_poses[i].quaternion[1] - median_poses[j].quaternion[1], 2);
	  total_dist += pow(median_poses[i].quaternion[2] - median_poses[j].quaternion[2], 2);
	  total_dist += pow(median_poses[i].quaternion[3] - median_poses[j].quaternion[3], 2);
	}
	if(i==0) min_dist = total_dist;
	else{
	  if(total_dist < min_dist){
	    min_dist = total_dist;
	    min_ind = i;
	  }
	}
      }
      ret_pose = median_poses[min_ind];
    }
  }
  
} //namespace

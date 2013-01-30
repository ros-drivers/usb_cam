/*
 Software License Agreement (BSD License)

 Copyright (c) 2012, Scott Niekum
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the Willow Garage nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 author: Scott Niekum
*/


#include "CvTestbed.h"
#include "MarkerDetector.h"
#include "Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

using namespace alvar;
using namespace std;

bool init=true;
Camera *cam;
cv_bridge::CvImagePtr cv_ptr_;
image_transport::Subscriber cam_sub_;
ros::Publisher arMarkerPub_;
ros::Publisher rvizMarkerPub_;
ar_track_alvar::AlvarMarkers arPoseMarkers_;
visualization_msgs::Marker rvizMarker_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;

double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic; 
std::string cam_info_topic; 
std::string output_frame;

void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg);


void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg)
{
	//If we've already gotten the cam info, then go ahead
	if(cam->getCamInfo_){
		try{
			tf::StampedTransform CamToOutput;
    			try{
					tf_listener->waitForTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, ros::Duration(1.0));
					tf_listener->lookupTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, CamToOutput);
   				}
    			catch (tf::TransformException ex){
      				ROS_ERROR("%s",ex.what());
    			}


            //Convert the image
            cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

            //Get the estimated pose of the main markers by using all the markers in each bundle

            // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
            // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
            // do this conversion here -jbinney
            IplImage ipl_image = cv_ptr_->image;

            marker_detector.Detect(&ipl_image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true);

			arPoseMarkers_.markers.clear ();
			for (size_t i=0; i<marker_detector.markers->size(); i++) 
			{
				//Get the pose relative to the camera
        		int id = (*(marker_detector.markers))[i].GetId(); 
				Pose p = (*(marker_detector.markers))[i].pose;
				double px = p.translation[0]/100.0;
				double py = p.translation[1]/100.0;
				double pz = p.translation[2]/100.0;
				double qx = p.quaternion[1];
				double qy = p.quaternion[2];
				double qz = p.quaternion[3];
				double qw = p.quaternion[0];

                tf::Quaternion rotation (qx,qy,qz,qw);
                tf::Vector3 origin (px,py,pz);
                tf::Transform t (rotation, origin);
                tf::Vector3 markerOrigin (0, 0, 0);
                tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
                tf::Transform markerPose = t * m; // marker pose in the camera frame

				//Publish the transform from the camera to the marker		
				std::string markerFrame = "ar_marker_";
				std::stringstream out;
				out << id;
				std::string id_string = out.str();
				markerFrame += id_string;
				tf::StampedTransform camToMarker (t, image_msg->header.stamp, image_msg->header.frame_id, markerFrame.c_str());
    			tf_broadcaster->sendTransform(camToMarker);
				
				//Create the rviz visualization messages
				tf::poseTFToMsg (markerPose, rvizMarker_.pose);
				rvizMarker_.header.frame_id = image_msg->header.frame_id;
				rvizMarker_.header.stamp = image_msg->header.stamp;
				rvizMarker_.id = id;

				rvizMarker_.scale.x = 1.0 * marker_size/100.0;
				rvizMarker_.scale.y = 1.0 * marker_size/100.0;
				rvizMarker_.scale.z = 0.2 * marker_size/100.0;
				rvizMarker_.ns = "basic_shapes";
				rvizMarker_.type = visualization_msgs::Marker::CUBE;
				rvizMarker_.action = visualization_msgs::Marker::ADD;
				switch (id)
				{
				  case 0:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 1.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 1:
				    rvizMarker_.color.r = 1.0f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 0.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 2:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 1.0f;
				    rvizMarker_.color.b = 0.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 3:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 0.5f;
				    rvizMarker_.color.b = 0.5f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 4:
				    rvizMarker_.color.r = 0.5f;
				    rvizMarker_.color.g = 0.5f;
				    rvizMarker_.color.b = 0.0;
				    rvizMarker_.color.a = 1.0;
				    break;
				  default:
				    rvizMarker_.color.r = 0.5f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 0.5f;
				    rvizMarker_.color.a = 1.0;
				    break;
				}
				rvizMarker_.lifetime = ros::Duration (1.0);
				rvizMarkerPub_.publish (rvizMarker_);

				//Get the pose of the tag in the camera frame, then the output frame (usually torso)				
				tf::Transform tagPoseOutput = CamToOutput * markerPose;

				//Create the pose marker messages
				ar_track_alvar::AlvarMarker ar_pose_marker;
				tf::poseTFToMsg (tagPoseOutput, ar_pose_marker.pose.pose);
      			ar_pose_marker.header.frame_id = output_frame;
			    ar_pose_marker.header.stamp = image_msg->header.stamp;
			    ar_pose_marker.id = id;
			    arPoseMarkers_.markers.push_back (ar_pose_marker);	
			}
			arMarkerPub_.publish (arPoseMarkers_);
		}
        catch (cv_bridge::Exception& e){
      		ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    	}
	}
}


int main(int argc, char *argv[])
{
	ros::init (argc, argv, "marker_detect");
	ros::NodeHandle n;
	
	if(argc < 7){
		std::cout << std::endl;
		cout << "Not enough arguments provided." << endl;
		cout << "Usage: ./individualMarkers <marker size in cm> <max new marker error> <max track error> <cam image topic> <cam info topic> <output frame>" << endl;
		std::cout << std::endl;
		return 0;
	}

	// Get params from command line
	marker_size = atof(argv[1]);
	max_new_marker_error = atof(argv[2]);
	max_track_error = atof(argv[3]);
	cam_image_topic = argv[4];
	cam_info_topic = argv[5];
    output_frame = argv[6];
	marker_detector.SetMarkerSize(marker_size);

	cam = new Camera(n, cam_info_topic);
	tf_listener = new tf::TransformListener(n);
	tf_broadcaster = new tf::TransformBroadcaster();
	arMarkerPub_ = n.advertise < ar_track_alvar::AlvarMarkers > ("ar_pose_marker", 0);
	rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
	
	//Give tf a chance to catch up before the camera callback starts asking for transforms
	ros::Duration(1.0).sleep();
	ros::spinOnce();	
	 
	ROS_INFO ("Subscribing to image topic");
	image_transport::ImageTransport it_(n);
    	cam_sub_ = it_.subscribe (cam_image_topic, 1, &getCapCallback);

	ros::spin ();

    return 0;
}

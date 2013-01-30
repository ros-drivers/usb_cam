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
#include "MultiMarkerBundle.h"
#include "MultiMarkerInitializer.h"
#include "Shared.h"
#include <cv_bridge/cv_bridge.h>.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

using namespace alvar;
using namespace std;

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

Camera *cam;
cv_bridge::CvImagePtr cv_ptr_;
image_transport::Subscriber cam_sub_;
ros::Publisher arMarkerPub_;
ros::Publisher rvizMarkerPub_;
ar_track_alvar::AlvarMarkers arPoseMarkers_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;
MultiMarkerBundle **multi_marker_bundles=NULL;
Pose *bundlePoses;
int *master_id;
bool *bundles_seen;
std::vector<int> *bundle_indices; 	
bool init = true;  

double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic; 
std::string cam_info_topic; 
std::string output_frame;
int n_bundles = 0;   

void GetMultiMarkerPoses(IplImage *image);
void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg);
void makeMarkerMsgs(int type, int id, Pose &p, sensor_msgs::ImageConstPtr image_msg, tf::StampedTransform &CamToOutput, visualization_msgs::Marker *rvizMarker, ar_track_alvar::AlvarMarker *ar_pose_marker);


// Updates the bundlePoses of the multi_marker_bundles by detecting markers and using all markers in a bundle to infer the master tag's position
void GetMultiMarkerPoses(IplImage *image) {

  if (marker_detector.Detect(image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true)){
    for(int i=0; i<n_bundles; i++)
      multi_marker_bundles[i]->Update(marker_detector.markers, cam, bundlePoses[i]);
    
    if(marker_detector.DetectAdditional(image, cam, false) > 0){
      for(int i=0; i<n_bundles; i++){
	if ((multi_marker_bundles[i]->SetTrackMarkers(marker_detector, cam, bundlePoses[i], image) > 0))
	  multi_marker_bundles[i]->Update(marker_detector.markers, cam, bundlePoses[i]);
      }
    }
  }
}


// Given the pose of a marker, builds the appropriate ROS messages for later publishing 
void makeMarkerMsgs(int type, int id, Pose &p, sensor_msgs::ImageConstPtr image_msg, tf::StampedTransform &CamToOutput, visualization_msgs::Marker *rvizMarker, ar_track_alvar::AlvarMarker *ar_pose_marker){
  double px,py,pz,qx,qy,qz,qw;
	
  px = p.translation[0]/100.0;
  py = p.translation[1]/100.0;
  pz = p.translation[2]/100.0;
  qx = p.quaternion[1];
  qy = p.quaternion[2];
  qz = p.quaternion[3];
  qw = p.quaternion[0];

  //Get the marker pose in the camera frame
  tf::Quaternion rotation (qx,qy,qz,qw);
  tf::Vector3 origin (px,py,pz);
  tf::Transform t (rotation, origin);  //transform from cam to marker

  tf::Vector3 markerOrigin (0, 0, 0);
  tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
  tf::Transform markerPose = t * m;

  //Publish the cam to marker transform for main marker in each bundle
  if(type==MAIN_MARKER){
    std::string markerFrame = "ar_marker_";
    std::stringstream out;
    out << id;
    std::string id_string = out.str();
    markerFrame += id_string;
    tf::StampedTransform camToMarker (t, image_msg->header.stamp, image_msg->header.frame_id, markerFrame.c_str());
    tf_broadcaster->sendTransform(camToMarker);
  }

  //Create the rviz visualization message
  tf::poseTFToMsg (markerPose, rvizMarker->pose);
  rvizMarker->header.frame_id = image_msg->header.frame_id;
  rvizMarker->header.stamp = image_msg->header.stamp;
  rvizMarker->id = id;

  rvizMarker->scale.x = 1.0 * marker_size/100.0;
  rvizMarker->scale.y = 1.0 * marker_size/100.0;
  rvizMarker->scale.z = 0.2 * marker_size/100.0;

  if(type==MAIN_MARKER)
    rvizMarker->ns = "main_shapes";
  else
    rvizMarker->ns = "basic_shapes";


  rvizMarker->type = visualization_msgs::Marker::CUBE;
  rvizMarker->action = visualization_msgs::Marker::ADD;

  //Determine a color and opacity, based on marker type
  if(type==MAIN_MARKER){
    rvizMarker->color.r = 1.0f;
    rvizMarker->color.g = 0.0f;
    rvizMarker->color.b = 0.0f;
    rvizMarker->color.a = 1.0;
  }
  else if(type==VISIBLE_MARKER){
    rvizMarker->color.r = 0.0f;
    rvizMarker->color.g = 1.0f;
    rvizMarker->color.b = 0.0f;
    rvizMarker->color.a = 0.7;
  }
  else if(type==GHOST_MARKER){
    rvizMarker->color.r = 0.0f;
    rvizMarker->color.g = 0.0f;
    rvizMarker->color.b = 1.0f;
    rvizMarker->color.a = 0.5;
  }

  rvizMarker->lifetime = ros::Duration (1.0);

  // Only publish the pose of the master tag in each bundle, since that's all we really care about aside from visualization 
  if(type==MAIN_MARKER){
    //Take the pose of the tag in the camera frame and convert to the output frame (usually torso_lift_link for the PR2)
    tf::Transform tagPoseOutput = CamToOutput * markerPose;

    //Create the pose marker message
    tf::poseTFToMsg (tagPoseOutput, ar_pose_marker->pose.pose);
    ar_pose_marker->header.frame_id = output_frame;
    ar_pose_marker->header.stamp = image_msg->header.stamp;
    ar_pose_marker->id = id;
  }
  else
    ar_pose_marker = NULL;
}


//Callback to handle getting video frames and processing them
void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg)
{
  //If we've already gotten the cam info, then go ahead
  if(cam->getCamInfo_){
    try{
      //Get the transformation from the Camera to the output frame for this image capture
      tf::StampedTransform CamToOutput;
      try{
	tf_listener->waitForTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, ros::Duration(1.0));
	tf_listener->lookupTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, CamToOutput);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
      }

      visualization_msgs::Marker rvizMarker;
      ar_track_alvar::AlvarMarker ar_pose_marker;
      arPoseMarkers_.markers.clear ();


      //Convert the image
      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

      //Get the estimated pose of the main markers by using all the markers in each bundle

      // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
      // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
      // do this conversion here -jbinney
      IplImage ipl_image = cv_ptr_->image;
      GetMultiMarkerPoses(&ipl_image);
		
      //Draw the observed markers that are visible and note which bundles have at least 1 marker seen
      for(int i=0; i<n_bundles; i++)
	bundles_seen[i] = false;

      for (size_t i=0; i<marker_detector.markers->size(); i++)
	{
	  int id = (*(marker_detector.markers))[i].GetId();

	  // Draw if id is valid
	  if(id >= 0){

	    //Mark the bundle that marker belongs to as "seen"
	    for(int j=0; j<n_bundles; j++){
	      for(int k=0; k<bundle_indices[j].size(); k++){
		if(bundle_indices[j][k] == id){
		  bundles_seen[j] = true;
		  break;
		}
	      }
	    }

	    // Don't draw if it is a master tag...we do this later, a bit differently
	    bool should_draw = true;
	    for(int i=0; i<n_bundles; i++){
	      if(id == master_id[i]) should_draw = false;
	    }
	    if(should_draw){
	      Pose p = (*(marker_detector.markers))[i].pose;
	      makeMarkerMsgs(VISIBLE_MARKER, id, p, image_msg, CamToOutput, &rvizMarker, &ar_pose_marker);
	      rvizMarkerPub_.publish (rvizMarker);
	    }
	  }
	}
			
      //Draw the main markers, whether they are visible or not -- but only if at least 1 marker from their bundle is currently seen
      for(int i=0; i<n_bundles; i++)
	{
	  if(bundles_seen[i] == true){
	    makeMarkerMsgs(MAIN_MARKER, master_id[i], bundlePoses[i], image_msg, CamToOutput, &rvizMarker, &ar_pose_marker);
	    rvizMarkerPub_.publish (rvizMarker);
	    arPoseMarkers_.markers.push_back (ar_pose_marker);
	  }
	}

      //Publish the marker messages
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

  if(argc < 8){
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./findMarkerBundles <marker size in cm> <max new marker error> <max track error> <cam image topic> <cam info topic> <output frame> <list of bundle XML files...>" << endl;
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
  int n_args_before_list = 7;
  n_bundles = argc - n_args_before_list;

  marker_detector.SetMarkerSize(marker_size);
  multi_marker_bundles = new MultiMarkerBundle*[n_bundles];	
  bundlePoses = new Pose[n_bundles];
  master_id = new int[n_bundles]; 
  bundle_indices = new std::vector<int>[n_bundles]; 
  bundles_seen = new bool[n_bundles]; 	

  // Load the marker bundle XML files
  for(int i=0; i<n_bundles; i++){	
    bundlePoses[i].Reset();		
    MultiMarker loadHelper;
    if(loadHelper.Load(argv[i + n_args_before_list], FILE_FORMAT_XML)){
      vector<int> id_vector = loadHelper.getIndices();
      multi_marker_bundles[i] = new MultiMarkerBundle(id_vector);	
      multi_marker_bundles[i]->Load(argv[i + n_args_before_list], FILE_FORMAT_XML);
      master_id[i] = multi_marker_bundles[i]->getMasterId();
      bundle_indices[i] = multi_marker_bundles[i]->getIndices();
    }
    else{
      cout<<"Cannot load file "<< argv[i + n_args_before_list] << endl;	
      return 0;
    }		
  }  

  // Set up camera, listeners, and broadcasters
  cam = new Camera(n, cam_info_topic);
  tf_listener = new tf::TransformListener(n);
  tf_broadcaster = new tf::TransformBroadcaster();
  arMarkerPub_ = n.advertise < ar_track_alvar::AlvarMarkers > ("ar_pose_marker", 0);
  rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
	
  //Give tf a chance to catch up before the camera callback starts asking for transforms
  ros::Duration(1.0).sleep();
  ros::spinOnce();			
	 
  //Subscribe to topics and set up callbacks
  ROS_INFO ("Subscribing to image topic");
  image_transport::ImageTransport it_(n);
  cam_sub_ = it_.subscribe (cam_image_topic, 1, &getCapCallback);

  ros::spin();

  return 0;
}

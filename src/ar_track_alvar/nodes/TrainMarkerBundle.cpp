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
#include <cv_bridge/cv_bridge.h>
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
MultiMarkerInitializer *multi_marker_init=NULL;
MultiMarkerBundle *multi_marker_bundle=NULL;
int auto_count;
bool auto_collect;

bool init=true;
bool add_measurement=false;
bool optimize = false;
bool optimize_done = false;

double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic; 
std::string cam_info_topic; 
std::string output_frame;
int nof_markers;  

double GetMultiMarkerPose(IplImage *image, Pose &pose);
void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg);
int keyCallback(int key);
void makeMarkerMsgs(int type, int id, Pose &p, sensor_msgs::ImageConstPtr image_msg, tf::StampedTransform &CamToOutput, visualization_msgs::Marker *rvizMarker, ar_track_alvar::AlvarMarker *ar_pose_marker);


double GetMultiMarkerPose(IplImage *image, Pose &pose) {
    static bool init=true;

    if (init) {
        init=false;
        vector<int> id_vector;
        for(int i = 0; i < nof_markers; ++i)
            id_vector.push_back(i);
        // We make the initialization for MultiMarkerBundle using MultiMarkerInitializer
        // Each marker needs to be visible in at least two images and at most 64 image are used.
        multi_marker_init = new MultiMarkerInitializer(id_vector, 2, 64);
        pose.Reset();
        multi_marker_init->PointCloudAdd(id_vector[0], marker_size, pose);
        multi_marker_bundle = new MultiMarkerBundle(id_vector);
		marker_detector.SetMarkerSize(marker_size); 
    }

    double error = -1;
    if (!optimize_done) {
        if (marker_detector.Detect(image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true)) {
        	error = multi_marker_init->Update(marker_detector.markers, cam, pose);
        }
    } 
	else {
    	if (marker_detector.Detect(image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true)) {
            error = multi_marker_bundle->Update(marker_detector.markers, cam, pose);
            if ((multi_marker_bundle->SetTrackMarkers(marker_detector, cam, pose, image) > 0) && (marker_detector.DetectAdditional(image, cam, false) > 0))
            {    
            	error = multi_marker_bundle->Update(marker_detector.markers, cam, pose);
            }
        }
    }

   if (add_measurement) {
		cout << "Markers seen: " << marker_detector.markers->size() << "\n";
        if (marker_detector.markers->size() >= 2) {
            cout<<"Adding measurement..."<<endl;
            multi_marker_init->MeasurementsAdd(marker_detector.markers);
        }
		else{
			cout << "Not enough markers to capture measurement\n";
    	}
		add_measurement=false;
	}

    if (optimize) {
        cout<<"Initializing..."<<endl;
        if (!multi_marker_init->Initialize(cam)) {
            cout<<"Initialization failed, add some more measurements."<<endl;

        } else {
            // Reset the bundle adjuster.
            multi_marker_bundle->Reset();
            multi_marker_bundle->MeasurementsReset();
            // Copy all measurements into the bundle adjuster.
            for (int i = 0; i < multi_marker_init->getMeasurementCount(); ++i) {
                Pose p2;
                multi_marker_init->getMeasurementPose(i, cam, p2);
                const std::vector<MultiMarkerInitializer::MarkerMeasurement> markers = multi_marker_init->getMeasurementMarkers(i);
                multi_marker_bundle->MeasurementsAdd(&markers, p2);
            }
            // Initialize the bundle adjuster with initial marker poses.
            multi_marker_bundle->PointCloudCopy(multi_marker_init);
            cout<<"Optimizing..."<<endl;
            if (multi_marker_bundle->Optimize(cam, 0.01, 20)) {
                cout<<"Optimizing done"<<endl;
                optimize_done=true;

            } else {
                cout<<"Optimizing FAILED!"<<endl;
            }
        }
        optimize=false;
    }
    return error;
}

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
	tf::Transform t (rotation, origin);

    tf::Vector3 markerOrigin (0, 0, 0);
	tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
	tf::Transform markerPose = t * m;

	//Publish the transform from the camera to the marker		
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
		rvizMarker->color.a = 1.0;
	}
	else if(type==GHOST_MARKER){
		rvizMarker->color.r = 0.0f;
		rvizMarker->color.g = 0.0f;
		rvizMarker->color.b = 1.0f;
		rvizMarker->color.a = 0.5;
	}

	rvizMarker->lifetime = ros::Duration (1.0);

	//Get the pose of the tag in the camera frame, then convert to the output frame (usually torso)
	tf::Transform tagPoseOutput = CamToOutput * markerPose;

	//Create the pose marker message
	tf::poseTFToMsg (tagPoseOutput, ar_pose_marker->pose.pose);
	ar_pose_marker->header.frame_id = output_frame;
	ar_pose_marker->header.stamp = image_msg->header.stamp;
	ar_pose_marker->id = id;
}


void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg)
{
	//Check if automatic measurement collection should be triggered
	if(auto_collect){
		auto_count++;
		add_measurement = true;
		if(auto_count >= 5)
			auto_collect = false;
	}

	//If we've already gotten the cam info, then go ahead
	if(cam->getCamInfo_){
		try{
			//Get the transformation from the Camera to the output frame for this image capture
			tf::StampedTransform CamToOutput;
    		try{
				tf_listener->waitForTransform(image_msg->header.frame_id, output_frame, image_msg->header.stamp, ros::Duration(1.0));
				tf_listener->lookupTransform(image_msg->header.frame_id, output_frame, image_msg->header.stamp, CamToOutput);				
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

            //Get the estimated pose of the main marker using the whole bundle
      		static Pose bundlePose;
            double error = GetMultiMarkerPose(&ipl_image, bundlePose);

			if (optimize_done){
    			//Draw the main marker
    			makeMarkerMsgs(MAIN_MARKER, 0, bundlePose, image_msg, CamToOutput, &rvizMarker, &ar_pose_marker);
    			rvizMarkerPub_.publish (rvizMarker);
    			arPoseMarkers_.markers.push_back (ar_pose_marker);
			}
    		//Now grab the poses of the other markers that are visible
			for (size_t i=0; i<marker_detector.markers->size(); i++)
			{
        		int id = (*(marker_detector.markers))[i].GetId();

        		//Don't do the main marker (id=0) if we've already drawn it
        		if(id > 0 || ((!optimize_done) && id==0)){
        			Pose p = (*(marker_detector.markers))[i].pose;
        			makeMarkerMsgs(VISIBLE_MARKER, id, p, image_msg, CamToOutput, &rvizMarker, &ar_pose_marker);
        			rvizMarkerPub_.publish (rvizMarker);
        			arPoseMarkers_.markers.push_back (ar_pose_marker);
        		}
			}
			arMarkerPub_.publish (arPoseMarkers_);
		}
        catch (cv_bridge::Exception& e){
      		ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    	}
	}

	//Sleep if we are auto collecting
	if(auto_collect)
		usleep(1000000);
}


//Do something based on keystrokes from menu
int keyProcess(int key)
{
    if(key == 'r')
    {
        cout<<"Reseting multi marker"<<endl;
            multi_marker_init->Reset();
            multi_marker_init->MeasurementsReset();
            multi_marker_bundle->Reset();
            multi_marker_bundle->MeasurementsReset();
            add_measurement = false;
            optimize        = false;
            optimize_done   = false;
    }
    else if(key == 'l')
    {
        if(multi_marker_bundle->Load("mmarker.xml", FILE_FORMAT_XML))
        {
            cout<<"Multi marker loaded"<<endl;
            multi_marker_init->PointCloudCopy(multi_marker_bundle);
            optimize_done = true;
        }
        else
            cout<<"Cannot load multi marker"<<endl;
    }
    else if(key == 's')
    {
        if(multi_marker_bundle->Save("mmarker.xml", FILE_FORMAT_XML))
            cout<<"Multi marker saved"<<endl;
        else
            cout<<"Cannot save multi marker"<<endl;
    }
    else if(key == 'p')
    {
        add_measurement=true;
    }
	else if(key == 'a')
    {
        auto_count = 0;
		auto_collect = true;
    }
    else if(key == 'o')
    {
        optimize=true;
    }
	else if(key == 'q')
    {
        exit(0);
    }
    else return key;

    return 0;
}


int main(int argc, char *argv[])
{
	ros::init (argc, argv, "marker_detect");
	ros::NodeHandle n;
	
	if(argc < 8){
		std::cout << std::endl;
		cout << "Not enough arguments provided." << endl;
		cout << "Usage: ./trainMarkerBundle <num of markers> <marker size in cm> <max new marker error> <max track error> <cam image topic> <cam info topic> <output frame>" << endl;
		std::cout << std::endl;
		return 0;
	}

	// Get params from command line
	nof_markers = atoi(argv[1]);
	marker_size = atof(argv[2]);
	max_new_marker_error = atof(argv[3]);
	max_track_error = atof(argv[4]);
	cam_image_topic = argv[5];
	cam_info_topic = argv[6];
    output_frame = argv[7];
	marker_detector.SetMarkerSize(marker_size);

	cam = new Camera(n, cam_info_topic);
	tf_listener = new tf::TransformListener(n);
	tf_broadcaster = new tf::TransformBroadcaster();
	arMarkerPub_ = n.advertise < ar_track_alvar::AlvarMarkers > ("ar_pose_marker", 0);
	rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
	
	//Give tf a chance to catch up before the camera callback starts asking for transforms
	ros::Duration(1.0).sleep();
	ros::spinOnce();

	//Subscribe to camera message
	ROS_INFO ("Subscribing to image topic");
	image_transport::ImageTransport it_(n);
    	cam_sub_ = it_.subscribe (cam_image_topic, 1, &getCapCallback);

    // Output usage message
    std::cout << std::endl;
    std::cout << "Keyboard Shortcuts:" << std::endl;
    std::cout << "  l: load marker configuration from mmarker.txt" << std::endl;
    std::cout << "  s: save marker configuration to mmarker.txt" << std::endl;
    std::cout << "  r: reset marker configuration" << std::endl;
    std::cout << "  p: add measurement" << std::endl;
	std::cout << "  a: auto add measurements (captures once a second for 5 seconds)" << std::endl;
    std::cout << "  o: optimize bundle" << std::endl;
    std::cout << "  q: quit" << std::endl;
    std::cout << std::endl;
    std::cout << "Please type commands with the openCV window selected" << std::endl;
	std::cout << std::endl;

	cvNamedWindow("Command input window", CV_WINDOW_AUTOSIZE); 

	while(1){
		int key = cvWaitKey(20);
		if(key >= 0)
			keyProcess(key);
		ros::spinOnce();
	}

    return 0;
}

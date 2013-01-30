#include "CvTestbed.h"
#include "MarkerDetector.h"
#include "MultiMarker.h"
#include "Pose.h"
#include "Shared.h"
using namespace alvar;
using namespace std;

int visualize=0;
bool detect_additional = false;
const int nof_markers = 5;
const double marker_size = 4;
MarkerDetector<MarkerData> marker_detector;
//MultiMarker<MarkerData> *multi_marker;
MultiMarker *multi_marker;
std::stringstream calibrationFilename;

void videocallback(IplImage *image)
{
    static Camera cam;
    Pose pose;
    bool flip_image = (image->origin?true:false);
    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }

    static bool init = true;
    if (init)
    {

        init = false;

        // Initialize camera
        cout<<"Loading calibration: "<<calibrationFilename.str();

        if (cam.SetCalib(calibrationFilename.str().c_str(), image->width, image->height))
        {
            cout<<" [Ok]"<<endl;
        }
        else
        {
            cam.SetRes(image->width, image->height);
            cout<<" [Fail]"<<endl;
        }

        vector<int> id_vector;
        for(int i = 0; i < nof_markers; ++i)
            id_vector.push_back(i);

        // We make the initialization for MultiMarkerBundle using a fixed marker field (can be printed from ALVAR.pdf)
        marker_detector.SetMarkerSize(marker_size);
        marker_detector.SetMarkerSizeForId(0, marker_size*2);
        multi_marker = new MultiMarker(id_vector);
        pose.Reset();
        multi_marker->PointCloudAdd(0, marker_size*2, pose);
        pose.SetTranslation(-marker_size*2.5, +marker_size*1.5, 0);
        multi_marker->PointCloudAdd(1, marker_size, pose);
        pose.SetTranslation(+marker_size*2.5, +marker_size*1.5, 0);
        multi_marker->PointCloudAdd(2, marker_size, pose);
        pose.SetTranslation(-marker_size*2.5, -marker_size*1.5, 0);
        multi_marker->PointCloudAdd(3, marker_size, pose);
        pose.SetTranslation(+marker_size*2.5, -marker_size*1.5, 0);
        multi_marker->PointCloudAdd(4, marker_size, pose);
    }

    double error=-1;
    if (marker_detector.Detect(image, &cam, true, (visualize == 1), 0.0)) {
        if (detect_additional) {
            error = multi_marker->Update(marker_detector.markers, &cam, pose);
            multi_marker->SetTrackMarkers(marker_detector, &cam, pose, visualize ? image : NULL);
            marker_detector.DetectAdditional(image, &cam, (visualize == 1));
        }
        if (visualize == 2) 
            error = multi_marker->Update(marker_detector.markers, &cam, pose, image);
        else
            error = multi_marker->Update(marker_detector.markers, &cam, pose);
    }

    static Marker foo;
    foo.SetMarkerSize(marker_size*4);
    if ((error >= 0) && (error < 5))
    {
        foo.pose = pose;
    }
    foo.Visualize(image, &cam);

    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }
}

int keycallback(int key)
{
    if(key == 'v')
    {
        visualize = (visualize+1)%3;
    }
    else if(key == 'l')
    {
        if(multi_marker->Load("mmarker.xml", alvar::FILE_FORMAT_XML))
        {
            cout<<"Multi marker loaded"<<endl;
        }
        else
            cout<<"Cannot load multi marker"<<endl;
    }
    else if(key == 'd')
    {
        detect_additional = !detect_additional;
        if(detect_additional)
            cout<<"Unreadable marker detection enabled."<<endl;
        else
            cout<<"Unreadable marker detection disabled."<<endl;
    }
    else return key;

    return 0;
}

int main(int argc, char *argv[])
{
    try {
        // Output usage message
        std::string filename(argv[0]);
        filename = filename.substr(filename.find_last_of('\\') + 1);
        std::cout << "SampleMultiMarker" << std::endl;
        std::cout << "=================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'MultiMarker' class to detect" << std::endl;
        std::cout << "  preconfigured multi-marker setup (see ALVAR.pdf). A large cube is drawn" << std::endl;
        std::cout << "  over the detected multi-marker even when only some of the markers are" << std::endl;
        std::cout << "  visible." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " [device]" << std::endl;
        std::cout << std::endl;
        std::cout << "    device    integer selecting device from enumeration list (default 0)" << std::endl;
        std::cout << "              highgui capture devices are prefered" << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  v: switch between three debug visualizations" << std::endl;
        std::cout << "  l: load marker configuration from mmarker.txt" << std::endl;
        std::cout << "  d: toggle the detection of non-readable markers" << std::endl;
        std::cout << "  q: quit" << std::endl;
        std::cout << std::endl;

        // Initialise CvTestbed
        CvTestbed::Instance().SetVideoCallback(videocallback);
        CvTestbed::Instance().SetKeyCallback(keycallback);

        // Enumerate possible capture plugins
        CaptureFactory::CapturePluginVector plugins = CaptureFactory::instance()->enumeratePlugins();
        if (plugins.size() < 1) {
            std::cout << "Could not find any capture plugins." << std::endl;
            return 0;
        }

        // Display capture plugins
        std::cout << "Available Plugins: ";
        outputEnumeratedPlugins(plugins);
        std::cout << std::endl;

        // Enumerate possible capture devices
        CaptureFactory::CaptureDeviceVector devices = CaptureFactory::instance()->enumerateDevices();
        if (devices.size() < 1) {
            std::cout << "Could not find any capture devices." << std::endl;
            return 0;
        }

        // Check command line argument for which device to use
        int selectedDevice = defaultDevice(devices);
        if (argc > 1) {
            selectedDevice = atoi(argv[1]);
        }
        if (selectedDevice >= (int)devices.size()) {
            selectedDevice = defaultDevice(devices);
        }
        
        // Display capture devices
        std::cout << "Enumerated Capture Devices:" << std::endl;
        outputEnumeratedDevices(devices, selectedDevice);
        std::cout << std::endl;
        
        // Create capture object from camera
        Capture *cap = CaptureFactory::instance()->createCapture(devices[selectedDevice]);
        std::string uniqueName = devices[selectedDevice].uniqueName();

        // Handle capture lifecycle and start video capture
        // Note that loadSettings/saveSettings are not supported by all plugins
        if (cap) {
            std::stringstream settingsFilename;
            settingsFilename << "camera_settings_" << uniqueName << ".xml";
            calibrationFilename << "camera_calibration_" << uniqueName << ".xml";
            
            cap->start();
            cap->setResolution(640, 480);
            
            if (cap->loadSettings(settingsFilename.str())) {
                std::cout << "Loading settings: " << settingsFilename.str() << std::endl;
            }

            std::stringstream title;
            title << "SampleMultiMarker (" << cap->captureDevice().captureType() << ")";

            CvTestbed::Instance().StartVideo(cap, title.str().c_str());

            if (cap->saveSettings(settingsFilename.str())) {
                std::cout << "Saving settings: " << settingsFilename.str() << std::endl;
            }

            cap->stop();
            delete cap;
        }
        else if (CvTestbed::Instance().StartVideo(0, argv[0])) {
        }
        else {
            std::cout << "Could not initialize the selected capture backend." << std::endl;
        }

        return 0;
    }
    catch (const std::exception &e) {
        std::cout << "Exception: " << e.what() << endl;
    }
    catch (...) {
        std::cout << "Exception: unknown" << std::endl;
    }
}

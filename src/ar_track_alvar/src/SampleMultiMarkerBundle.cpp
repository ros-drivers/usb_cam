#include "CvTestbed.h"
#include "MarkerDetector.h"
#include "MultiMarkerBundle.h"
#include "MultiMarkerInitializer.h"
#include "Pose.h"
#include "Shared.h"
using namespace alvar;
using namespace std;

int visualize=1;
const int nof_markers = 8;
const double marker_size = 4;
bool add_measurement=false;
bool optimize = false;
bool optimize_done = false;
MarkerDetector<MarkerData> marker_detector;
std::stringstream calibrationFilename;

MultiMarkerInitializer *multi_marker_init=NULL;
MultiMarkerBundle *multi_marker_bundle=NULL;
double GetMultiMarkerPose(IplImage *image, Camera *cam, Pose &pose) {
    static bool init=true;
    if (init) {
        cout<<"Using manual multimarker approach with MultiMarkerInitializer & MultiMarkerBundle."<<endl;
        cout<<"Use 'p' for taking keyframes and 'o' for optimizing."<<endl;
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
    }

    double error = -1;
    if (!optimize_done) {
        if (marker_detector.Detect(image, cam, true, (visualize == 1), 0.0)) {
            if (visualize == 2)
                error = multi_marker_init->Update(marker_detector.markers, cam, pose, image);
            else
                error = multi_marker_init->Update(marker_detector.markers, cam, pose);
        }
    } else {
        if (marker_detector.Detect(image, cam, true, (visualize == 1), 0.0)) {
            if (visualize == 2)
                error = multi_marker_bundle->Update(marker_detector.markers, cam, pose, image);
            else 
                error = multi_marker_bundle->Update(marker_detector.markers, cam, pose);
            if ((multi_marker_bundle->SetTrackMarkers(marker_detector, cam, pose, image) > 0) &&
                (marker_detector.DetectAdditional(image, cam, (visualize == 3)) > 0))
            {
                if (visualize == 3)
                    error = multi_marker_bundle->Update(marker_detector.markers, cam, pose, image);
                else
                    error = multi_marker_bundle->Update(marker_detector.markers, cam, pose);
            }
        }
    }

    if (add_measurement) {
        if (marker_detector.markers->size() >= 2) {
            cout<<"Adding measurement..."<<endl;
            multi_marker_init->MeasurementsAdd(marker_detector.markers);
            add_measurement=false;
        }
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
                Pose pose;
                multi_marker_init->getMeasurementPose(i, cam, pose);
                const std::vector<MultiMarkerInitializer::MarkerMeasurement> markers 
                    = multi_marker_init->getMeasurementMarkers(i);
                multi_marker_bundle->MeasurementsAdd(&markers, pose);
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

void videocallback(IplImage *image)
{
    static Camera cam;
    static Pose pose;
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

        marker_detector.SetMarkerSize(marker_size);
    }

    // Manual approach (use 'p' for taking keyframes and 'o' for optimizing)
    double error = GetMultiMarkerPose(image, &cam, pose);

    // Visualize a blue marker at the origo.
    static Marker foo;
    foo.SetMarkerSize(marker_size);
    if ((error >= 0) && (error < 5))
    {
        foo.pose = pose;
    }
    foo.Visualize(image, &cam, CV_RGB(0,0,255));

    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }
}

int keycallback(int key)
{
    static bool fixed = false;

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
    else if(key == 'v')
    {
        visualize = (visualize+1)%3;
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
    else if(key == 'o')
    {
        optimize=true;
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
        std::cout << "SampleMultiMarkerBundle" << std::endl;
        std::cout << "=======================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'MultiMarkerBundle' class to" << std::endl;
        std::cout << "  automatically deduce and optimize 'MultiMarker' setups." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " [device]" << std::endl;
        std::cout << std::endl;
        std::cout << "    device    integer selecting device from enumeration list (default 0)" << std::endl;
        std::cout << "              highgui capture devices are prefered" << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  l: load marker configuration from mmarker.txt" << std::endl;
        std::cout << "  s: save marker configuration to mmarker.txt" << std::endl;
        std::cout << "  r: reset marker configuration" << std::endl;
        std::cout << "  p: add measurement" << std::endl;
        std::cout << "  o: optimize bundle" << std::endl;
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
            title << "SampleMultiMarkerBundle (" << cap->captureDevice().captureType() << ")";

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

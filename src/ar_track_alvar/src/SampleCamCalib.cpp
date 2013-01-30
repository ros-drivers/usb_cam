#include <iostream>
#include "Camera.h"
#include "CvTestbed.h"
#include "Shared.h"
using namespace alvar;
using namespace std;

const int calib_count_max=50;
const int etalon_rows=6;
const int etalon_columns=8;
std::stringstream calibrationFilename;

void videocallback(IplImage *image)
{
    static bool calibrated = false;
    static int calib_count=0;
    static Camera cam;
    static ProjPoints pp;
    static int64 prev_tick=0;
    static bool initialized = false;

    if (!initialized) {
      cam.SetRes(image->width, image->height);
      prev_tick = cvGetTickCount();
      initialized = true;
    }
  
    bool flip_image = (image->origin?true:false);
    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }

    assert(image);
    if (!calibrated) {
        // If we have already collected enough data to make the calibration
        // - We are ready to end the capture loop
        // - Calibrate
        // - Save the calibration file
        if (calib_count >= calib_count_max) {
            std::cout<<"Calibrating..."<<endl;
            calib_count = 0;
            cam.Calibrate(pp);
            pp.Reset();
            cam.SaveCalib(calibrationFilename.str().c_str());
            std::cout<<"Saving calibration: "<<calibrationFilename.str()<<endl;
            calibrated = true;
        } 
        // If we are still collecting calibration data
        // - For every 1.5s add calibration data from detected 7*9 chessboard (and visualize it if true)
        else {
            int64 tick = cvGetTickCount();
            if ((tick - prev_tick) > (cvGetTickFrequency() * 1000 * 1000 * 1.5)) {
                if (pp.AddPointsUsingChessboard(image, 2.8, etalon_rows, etalon_columns, true)) {
                    prev_tick = tick;
                    calib_count++;
                    cout<<calib_count<<"/"<<calib_count_max<<endl;
                }
            }
        }
    } else {
        if (pp.AddPointsUsingChessboard(image, 2.5, etalon_rows, etalon_columns, true)) {
            Pose pose;
            cam.CalcExteriorOrientation(pp.object_points, pp.image_points, &pose);
            cam.ProjectPoints(pp.object_points, &pose, pp.image_points);
            for (size_t i=0; i<pp.image_points.size(); i++) {
                cvCircle(image, cvPoint((int)pp.image_points[i].x, (int)pp.image_points[i].y), 6, CV_RGB(0, 0, 255));
            }
            pp.Reset();
        }
    }

    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }
}

int main(int argc, char *argv[])
{
    try {
        // Output usage message
        std::string filename(argv[0]);
        filename = filename.substr(filename.find_last_of('\\') + 1);
        std::cout << "SampleCamCalib" << std::endl;
        std::cout << "==============" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'Camera' and 'ProjPoints' classes" << std::endl;
        std::cout << "  to perform camera calibration. Point the camera to the chessboard" << std::endl;
        std::cout << "  calibration pattern (see ALVAR.pdf) from several directions until 50" << std::endl;
        std::cout << "  calibration images are collected. A 'calib.xml' file that contains the" << std::endl;
        std::cout << "  internal parameters of the camera is generated and can be used by other" << std::endl;
        std::cout << "  applications that require a calibrated camera." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " [device]" << std::endl;
        std::cout << std::endl;
        std::cout << "    device    integer selecting device from enumeration list (default 0)" << std::endl;
        std::cout << "              highgui capture devices are prefered" << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  q: quit" << std::endl;
        std::cout << std::endl;

        // Initialise CvTestbed
        CvTestbed::Instance().SetVideoCallback(videocallback);

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
            title << "SampleCamCalib (" << cap->captureDevice().captureType() << ")";

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

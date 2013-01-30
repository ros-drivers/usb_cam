#include "CvTestbed.h"
#include "GlutViewer.h"
#include "Shared.h"
#include "FernImageDetector.h"
#include "FernPoseEstimator.h"

using namespace alvar;
using namespace std;

bool init = true;
std::stringstream calibrationFilename;
FernPoseEstimator fernEstimator;
FernImageDetector fernDetector(true);
Drawable d;
cv::Mat gray;
bool reset = false;

void videocallback(IplImage *image)
{
    bool flip_image = (image->origin?true:false);
    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }

    if (init) {
        init = false;
        cout << "Loading calibration: " << calibrationFilename.str();
        if (fernEstimator.setCalibration(calibrationFilename.str(), image->width, image->height)) {
            cout << " [Ok]" << endl;
        } else {
            fernEstimator.setResolution(image->width, image->height);
            cout << " [Fail]" << endl;
        }
        double p[16];
        fernEstimator.camera().GetOpenglProjectionMatrix(p, image->width, image->height);
        GlutViewer::SetGlProjectionMatrix(p);
        d.SetScale(10);
        gray = cv::Mat(image);
    }

    if (image->nChannels == 3) {
        cv::Mat img = cvarrToMat(image);
        cv::cvtColor(img, gray, CV_RGB2GRAY);
    }
    else {
        gray = image;
    }

    vector<CvPoint2D64f> ipts;
    vector<CvPoint3D64f> mpts;

    fernDetector.findFeatures(gray, true);
    fernDetector.imagePoints(ipts);
    fernDetector.modelPoints(mpts, true);
    double test = fernDetector.inlierRatio();
    if (test > 0.15 && mpts.size() > 4) {
        fernEstimator.calculateFromPointCorrespondences(mpts, ipts);
    }

    GlutViewer::DrawableClear();
    Pose pose = fernEstimator.pose();
    pose.GetMatrixGL(d.gl_mat);
    GlutViewer::DrawableAdd(&d);

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
        std::cout << "SampleMarkerlessDetector" << std::endl;
        std::cout << "========================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'FernImageDetector' and" << std::endl;
        std::cout << "  'FernPoseEstimator' classes to detect and track an image and" << std::endl;
        std::cout << "  visualize it using 'GlutViewer'. The classification must first" << std::endl;
        std::cout << "  be trained with the SampleMarkerlessCreator sample and the" << std::endl;
        std::cout << "  resulting file passed as an argument to this sample." << std::endl;
        std::cout << std::endl;
        std::cout << "  For optimal results, a high quality USB camera or a Firewire" << std::endl;
        std::cout << "  camera is necessary. It is also advised to calibrate the camera" << std::endl;
        std::cout << "  using the SampleCamCalib sample. It should be noted that the size" << std::endl;
        std::cout << "  of the trained image will affect the optimal distance for detection." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " filename [device]" << std::endl;
        std::cout << std::endl;
        std::cout << "    filename the filename of classifier (.dat)" << std::endl;
        std::cout << "    device   integer selecting device from enumeration list (default 0)" << std::endl;
        std::cout << "             highgui capture devices are prefered" << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  q: quit" << std::endl;
        std::cout << std::endl;

        if (argc < 2) {
            std::cout << "Filename not specified." << std::endl;
            return 0;
        }
        std::string classifierFilename(argv[1]);

        // Initialise GlutViewer and CvTestbed
        GlutViewer::Start(argc, argv, 640, 480);
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
        if (argc > 2) {
            selectedDevice = atoi(argv[2]);
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
            std::cout << "Loading classifier." << std::endl;
            if (!fernDetector.read(classifierFilename)) {
                std::cout << "Loading classifier failed." << std::endl;
                cap->stop();
                delete cap;
                return 1;
            }

            std::stringstream settingsFilename;
            settingsFilename << "camera_settings_" << uniqueName << ".xml";
            calibrationFilename << "camera_calibration_" << uniqueName << ".xml";
            
            cap->start();
            cap->setResolution(640, 480);
            
            if (cap->loadSettings(settingsFilename.str())) {
                std::cout << "Loading settings: " << settingsFilename.str() << std::endl;
            }

            std::stringstream title;
            title << "SampleMarkerDetector (" << cap->captureDevice().captureType() << ")";

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

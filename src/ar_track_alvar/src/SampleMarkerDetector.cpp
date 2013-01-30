#include "CvTestbed.h"
#include "MarkerDetector.h"
#include "GlutViewer.h"
#include "Shared.h"
using namespace alvar;
using namespace std;

bool init=true;
const int marker_size=15;
Camera cam;
Drawable d[32];
std::stringstream calibrationFilename;

void videocallback(IplImage *image)
{
    static IplImage *rgba;
    bool flip_image = (image->origin?true:false);
    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }

    if (init) {
        init = false;
        cout<<"Loading calibration: "<<calibrationFilename.str();
        if (cam.SetCalib(calibrationFilename.str().c_str(), image->width, image->height)) {
            cout<<" [Ok]"<<endl;
        } else {
            cam.SetRes(image->width, image->height);
            cout<<" [Fail]"<<endl;
        }
        double p[16];
        cam.GetOpenglProjectionMatrix(p,image->width,image->height);
        GlutViewer::SetGlProjectionMatrix(p);
        for (int i=0; i<32; i++) {
            d[i].SetScale(marker_size);
        }
        rgba = CvTestbed::Instance().CreateImageWithProto("RGBA", image, 0, 4);
    }
    static MarkerDetector<MarkerData> marker_detector;
    marker_detector.SetMarkerSize(marker_size); // for marker ids larger than 255, set the content resolution accordingly
    //static MarkerDetector<MarkerArtoolkit> marker_detector;
    //marker_detector.SetMarkerSize(2.8, 3, 1.5);

    // Here we try to use RGBA just to make sure that also it works...
    //cvCvtColor(image, rgba, CV_RGB2RGBA);
    marker_detector.Detect(image, &cam, true, true);
    GlutViewer::DrawableClear();
    for (size_t i=0; i<marker_detector.markers->size(); i++) {
        if (i >= 32) break;
        
        Pose p = (*(marker_detector.markers))[i].pose;
        p.GetMatrixGL(d[i].gl_mat);

        int id = (*(marker_detector.markers))[i].GetId();
        double r = 1.0 - double(id+1)/32.0;
        double g = 1.0 - double(id*3%32+1)/32.0;
        double b = 1.0 - double(id*7%32+1)/32.0;
        d[i].SetColor(r, g, b);

        GlutViewer::DrawableAdd(&(d[i]));
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
        std::cout << "SampleMarkerDetector" << std::endl;
        std::cout << "====================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to detect 'MarkerData' markers using" << std::endl;
        std::cout << "  'MarkerDetector' and visualize them using 'GlutViewer'. In the" << std::endl;
        std::cout << "  SampleMarkerDetector window, various debug information is shown" << std::endl;
        std::cout << "  about the detected markers. The coordinate axes and a virtual cube" << std::endl;
        std::cout << "  are superimposed onto the markers to visualize the detected pose." << std::endl;
        std::cout << "  For each marker, a small image of the marker content is displayed" << std::endl;
        std::cout << "  at the origin and the marker number is displayed at one of the" << std::endl;
        std::cout << "  corners. At the opposing corner, the error estimation percentages" << std::endl;
        std::cout << "  'MARGIN_ERROR' and 'DECODE_ERROR' (red) or 'TRACK_ERROR' (dark red)" << std::endl;
        std::cout << "  are displayed." << std::endl;
        std::cout << std::endl;
        std::cout << "  In the AR window, squares are drawn over the marker positions using" << std::endl;
        std::cout << "  OpenGL. In the VR window, the squares are drawn with respect to the" << std::endl;
        std::cout << "  camera coordinate frame. The viewpoint can be modified by dragging" << std::endl;
        std::cout << "  with the left and right mouse buttons." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " [device|filename]" << std::endl;
        std::cout << std::endl;
        std::cout << "    device    integer selecting device from enumeration list (default 0)" << std::endl;
        std::cout << "              highgui capture devices are prefered" << std::endl;
        std::cout << "    filename  string specifying a media file as input" << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  q: quit" << std::endl;
        std::cout << std::endl;

        // Initialise GlutViewer and CvTestbed
        GlutViewer::Start(argc, argv, 640, 480);
        CvTestbed::Instance().SetVideoCallback(videocallback);

        // Create capture object from camera (argv[1] is a number) or from file (argv[1] is a string)
        Capture *cap;
        std::string uniqueName;
        if ((argc > 1) && (!isdigit(argv[1][0]))) {
            // Manually create capture device and initialize capture object
            CaptureDevice device("file", argv[1]);
            cap = CaptureFactory::instance()->createCapture(device);
            uniqueName = "file";
        }
        else {    
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
            cap = CaptureFactory::instance()->createCapture(devices[selectedDevice]);
            uniqueName = devices[selectedDevice].uniqueName();
        }

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

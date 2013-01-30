#include "CvTestbed.h"
#include "Shared.h"
using namespace alvar;
using namespace std;

void videocallback(IplImage *image)
{
    static IplImage *img_gray=NULL;

    assert(image);
    if (img_gray == NULL) {
        // Following image is toggled visible using key '0'
        img_gray = CvTestbed::Instance().CreateImageWithProto("Grayscale", image, 0, 1);
    }
    if (image->nChannels > 1) { 
        cvCvtColor(image, img_gray, CV_RGB2GRAY);
    } else {
        cvCopy(image, img_gray);
    }
    // TODO: Do your image operations
}

int main(int argc, char *argv[])
{
    try {
        // Output usage message
        std::string filename(argv[0]);
        filename = filename.substr(filename.find_last_of('\\') + 1);
        std::cout << "SampleCvTestbed" << std::endl;
        std::cout << "===============" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'CvTestbed', 'CaptureFactory' and" << std::endl;
        std::cout << "  'Capture' classes. The 'CaptureFactory' can create 'Capture' objects" << std::endl;
        std::cout << "  from many different backends (see SampleCvTestbed.cpp). You can also" << std::endl;
        std::cout << "  show/hide the 1st ten images created using 'CvTestbed' using the number" << std::endl;
        std::cout << "  keys. In this example you can use key '0' to show/hide a grayscale" << std::endl;
        std::cout << "  version of the captured image." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " [device]" << std::endl;
        std::cout << std::endl;
        std::cout << "    device    integer selecting device from enumeration list (default 0)" << std::endl;
        std::cout << "              highgui capture devices are prefered" << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  0: show/hide grayscale image" << std::endl;
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

            cap->start();
            cap->setResolution(640, 480);

            if (cap->loadSettings(settingsFilename.str())) {
                std::cout << "Loading settings: " << settingsFilename.str() << std::endl;
            }

            std::stringstream title;
            title << "SampleCvTestbed (" << cap->captureDevice().captureType() << ")";

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

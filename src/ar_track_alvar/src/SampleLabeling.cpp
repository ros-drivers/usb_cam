#include "CvTestbed.h"
#include "ConnectedComponents.h"
#include "Shared.h"
using namespace alvar;
using namespace std;

int thresh_param1 = 31;

void videocallback(IplImage *image)
{
    bool flip_image = (image->origin?true:false);
    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }

    static LabelingCvSeq* labeling = 0;
    if(!labeling) {
        labeling = new LabelingCvSeq();
    }

    labeling->SetThreshParams(thresh_param1, 5);

    const int min_edge_size = 10;
    CvSeq* edges = labeling->LabelImage(image, min_edge_size);

    int n_edges = edges->total;
    for(int i = 0; i < n_edges; ++i)
    {
        CvSeq* pixels = (CvSeq*)cvGetSeqElem(edges, i);
        int n_pixels = pixels->total;
        for(int j = 0; j < n_pixels; ++j)
        {
            CvPoint* pt = (CvPoint*)cvGetSeqElem(pixels, j);
            cvLine(image, *pt, *pt, CV_RGB(255,0,0));
        }
    }

    // Visualize now also the square corners.
    labeling->LabelSquares(image, true);

    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }
}

int keycallback(int key)
{
    if(key == '+')
    {
        thresh_param1+=2;
        std::cout<<"Adaptive threshold block size: "<<thresh_param1<<std::endl;
    }
    else if(key == '-')
    {
        thresh_param1-=2;
        if(thresh_param1<3) thresh_param1 = 3;
        std::cout<<"Adaptive threshold block size: "<<thresh_param1<<std::endl;
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
        std::cout << "SampleLabeling" << std::endl;
        std::cout << "==============" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'LabelingCvSeq' class to perform" << std::endl;
        std::cout << "  labeling. Blobs are detected in the image and if the blobs have four" << std::endl;
        std::cout << "  corners, the edges between the corners are visualized." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " [device]" << std::endl;
        std::cout << std::endl;
        std::cout << "    device    integer selecting device from enumeration list (default 0)" << std::endl;
        std::cout << "              highgui capture devices are prefered" << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  +: Increase adaptive threshold block size." << std::endl;
        std::cout << "  -: Decrease adaptive threshold block size." << std::endl;
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
            
            cap->start();
            cap->setResolution(640, 480);
            
            if (cap->loadSettings(settingsFilename.str())) {
                std::cout << "Loading settings: " << settingsFilename.str() << std::endl;
            }

            std::stringstream title;
            title << "SampleLabeling (" << cap->captureDevice().captureType() << ")";

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

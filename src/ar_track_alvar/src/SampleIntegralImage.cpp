#include "CvTestbed.h"
#include "IntegralImage.h"
#include "Shared.h"
using namespace alvar;
using namespace std;

void videocallback(IplImage *image)
{
    static IplImage *img_grad = NULL;
    static IplImage *img_gray = NULL;
    static IplImage *img_ver = NULL;
    static IplImage *img_hor = NULL;
    static IplImage *img_canny = NULL;
    static IntegralImage integ;
    static IntegralGradient grad;

    assert(image);
    if (img_gray == NULL) {
        // Following image is toggled visible using key '0'
        img_grad = CvTestbed::Instance().CreateImageWithProto("Gradient", image);
        CvTestbed::Instance().ToggleImageVisible(0);
        img_gray = CvTestbed::Instance().CreateImageWithProto("Grayscale", image, 0, 1);
        img_ver = CvTestbed::Instance().CreateImage("Vertical", cvSize(1,image->height), IPL_DEPTH_8U, 1);
        img_hor = CvTestbed::Instance().CreateImage("Horizontal", cvSize(image->width,1), IPL_DEPTH_8U, 1);
        img_canny = CvTestbed::Instance().CreateImageWithProto("Canny", image, 0, 1);
        img_canny->origin = img_ver->origin = img_hor->origin = image->origin;
    }
    if (image->nChannels > 1) { 
        cvCvtColor(image, img_gray, CV_RGB2GRAY);
    } else {
        cvCopy(image, img_gray);
    }

    // Show PerformanceTimer
    //PerformanceTimer timer;
    //timer.Start();

    // Update the integral images
    integ.Update(img_gray);
    grad.Update(img_gray);

    // Whole image projections
    integ.GetSubimage(cvRect(0,0,image->width,image->height), img_ver);
    integ.GetSubimage(cvRect(0,0,image->width,image->height), img_hor);
    for (int y=1; y<image->height; y++) {
        cvLine(image, 
               cvPoint(int(cvGet2D(img_ver, y-1, 0).val[0]), y-1), 
               cvPoint(int(cvGet2D(img_ver, y, 0).val[0]), y), 
               CV_RGB(255,0,0));
    }
    for (int x=1; x<image->width; x++) {
        cvLine(image, 
               cvPoint(x-1, int(cvGet2D(img_hor, 0, x-1).val[0])), 
               cvPoint(x, int(cvGet2D(img_hor, 0, x).val[0])), 
               CV_RGB(0,255,0));
    }

    // Gradients
    // Mark gradients for 4x4 sub-blocks
    /*
    cvZero(img_grad);
    CvRect r = {0,0,4,4};
    for (int y=0; y<image->height/4; y++) {
        r.y = y*4;
        for (int x=0; x<image->width/4; x++) {
            r.x = x*4;
            double dirx, diry;
            grad.GetAveGradient(r, &dirx, &diry);
            cvLine(img_grad, cvPoint(r.x+2,r.y+2), cvPoint(r.x+2+int(dirx),r.y+2+int(diry)), CV_RGB(255,0,0));
        }
    }
    */

    // Gradients on canny
    cvZero(img_grad);
    static int t1=64, t2=192;
    cvCreateTrackbar("t1", "Gradient", &t1, 255, NULL);
    cvCreateTrackbar("t2", "Gradient", &t2, 255, NULL);
    cvCanny(img_gray, img_canny, t1, t2);
    CvRect r = {0,0,4,4};
    for (r.y=0; r.y<img_canny->height-4; r.y++) {
        for (r.x=0; r.x<img_canny->width-4; r.x++) {
            if (img_canny->imageData[r.y*img_canny->widthStep+r.x]) {
                double dirx, diry;
                grad.GetAveGradient(r, &dirx, &diry);
                cvLine(img_grad, cvPoint(r.x+2,r.y+2), cvPoint(r.x+2+int(dirx),r.y+2+int(diry)), CV_RGB(0,0,255));
                cvLine(img_grad, cvPoint(r.x+2,r.y+2), cvPoint(r.x+2+int(-diry),r.y+2+int(+dirx)), CV_RGB(255,0,0));
                cvLine(img_grad, cvPoint(r.x+2,r.y+2), cvPoint(r.x+2+int(+diry),r.y+2+int(-dirx)), CV_RGB(255,0,0));
            }
        }
    }

    // Show PerformanceTimer
    //cout<<"Processing: "<<1.0 / timer.Stop()<<" fps"<<endl;
}

int main(int argc, char *argv[])
{
    try {
        // Output usage message
        std::string filename(argv[0]);
        filename = filename.substr(filename.find_last_of('\\') + 1);
        std::cout << "SampleIntegralImage" << std::endl;
        std::cout << "===================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'IntegralImage' and" << std::endl;
        std::cout << "  'IntegralGradient' classes. The vertical (green) and horizontal (red)" << std::endl;
        std::cout << "  whole image projections are computed using 'IntegralImage::GetSubimage'" << std::endl;
        std::cout << "  and shown in the SampleIntegralImage window. The gradients of the" << std::endl;
        std::cout << "  image edges are shown in the Gradient window. The edges are detected" << std::endl;
        std::cout << "  using the Canny edge detector where t1 and t2 are parameters for the" << std::endl;
        std::cout << "  Canny algorithm. The gradients are drawn in red and their local normals" << std::endl;
        std::cout << "  are drawn in blue." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " [device]" << std::endl;
        std::cout << std::endl;
        std::cout << "    device    integer selecting device from enumeration list (default 0)" << std::endl;
        std::cout << "              highgui capture devices are prefered" << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  0: show/hide gradient image" << std::endl;
        std::cout << "  1: show/hide grayscale image" << std::endl;
        std::cout << "  2: show/hide vertical image" << std::endl;
        std::cout << "  3: show/hide horizontal image" << std::endl;
        std::cout << "  4: show/hide canny image" << std::endl;
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
            title << "SampleIntegralImage (" << cap->captureDevice().captureType() << ")";

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

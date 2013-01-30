#include "CvTestbed.h"
#include "MarkerDetector.h"
#include "GlutViewer.h"
#include "Shared.h"
using namespace alvar;
using namespace std;

#define GLUT_DISABLE_ATEXIT_HACK // Needed to compile with Mingw?
#include <GL/gl.h>

const double margin = 1.0;
std::stringstream calibrationFilename;

// Own drawable for showing hide-texture in OpenGL
struct OwnDrawable : public Drawable {
    unsigned char hidingtex[64*64*4];
    virtual void Draw() {
        glPushMatrix();
        glMultMatrixd(gl_mat);
        
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glEnable(GL_TEXTURE_2D);
        int tex=0;
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,64,64,0,GL_RGBA,GL_UNSIGNED_BYTE,hidingtex);
        glDisable(GL_CULL_FACE);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_ALPHA_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBegin(GL_QUADS);
            glTexCoord2d(0.0,0.0);
            glVertex3d(-margin,-margin,0);
            glTexCoord2d(0.0,1.0);
            glVertex3d(-margin,margin,0);
            glTexCoord2d(1.0,1.0);
            glVertex3d(margin,margin,0);
            glTexCoord2d(1.0,0.0);
            glVertex3d(margin,-margin,0);
        glEnd();
        glPopAttrib();
        glPopMatrix();
    }
};

void videocallback(IplImage *image)
{
    static bool init=true;
    static const int marker_size=15;
    static Camera cam;
    static OwnDrawable d[32];
    static IplImage *hide_texture;

    bool flip_image = (image->origin?true:false);
    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }

    static IplImage* bg_image = 0;
    if(!bg_image) bg_image = cvCreateImage(cvSize(512, 512), 8, 3);
    if(image->nChannels == 3)
    {
        bg_image->origin = 0;
        cvResize(image, bg_image);
        GlutViewer::SetVideo(bg_image);
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
        hide_texture = CvTestbed::Instance().CreateImage("hide_texture", cvSize(64, 64), 8, 4);
    }
    static MarkerDetector<MarkerData> marker_detector;\
    marker_detector.Detect(image, &cam, false, false);

    GlutViewer::DrawableClear();
    for (size_t i=0; i<marker_detector.markers->size(); i++) {
        if (i >= 32) break;
        GlutViewer::DrawableAdd(&(d[i]));
    }
    for (size_t i=0; i<marker_detector.markers->size(); i++) {
        if (i >= 32) break;
        
        // Note that we need to mirror both the y- and z-axis because:
        // - In OpenCV we have coordinates: x-right, y-down, z-ahead
        // - In OpenGL we have coordinates: x-right, y-up, z-backwards
        // TODO: Better  option might be to use OpenGL projection matrix that matches our OpenCV-approach
        Pose p = (*(marker_detector.markers))[i].pose;
        BuildHideTexture(image, hide_texture, &cam, d[i].gl_mat, PointDouble(-margin, -margin), PointDouble(margin, margin));
        //DrawTexture(image, hide_texture, &cam, d[i].gl_mat, PointDouble(-0.7, -0.7), PointDouble(0.7, 0.7));
        
        p.GetMatrixGL(d[i].gl_mat);
        for (int ii=0; ii<64*64; ii++) {
            d[i].hidingtex[ii*4+0] = hide_texture->imageData[ii*4+2];
            d[i].hidingtex[ii*4+1] = hide_texture->imageData[ii*4+1];
            d[i].hidingtex[ii*4+2] = hide_texture->imageData[ii*4+0];
            d[i].hidingtex[ii*4+3] = hide_texture->imageData[ii*4+3];
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
        std::cout << "SampleMarkerHide" << std::endl;
        std::cout << "================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to detect 'MarkerData' markers, similarly" << std::endl;
        std::cout << "  to 'SampleMarkerDetector', and hide them using the 'BuildHideTexture'" << std::endl;
        std::cout << "  and 'DrawTexture' classes." << std::endl;
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

        // Initialise GlutViewer and CvTestbed
        GlutViewer::Start(argc, argv, 640, 480, 15);
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
            title << "SampleMarkerHide (" << cap->captureDevice().captureType() << ")";

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

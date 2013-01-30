#include "CvTestbed.h"
#include "GlutViewer.h"
#include "SfM.h"
#include "Shared.h"
using namespace alvar;
using namespace std;

const double marker_size=1;
bool init=true;
SimpleSfM *sfm;
std::stringstream calibrationFilename;

// Own drawable 3D cross on features
struct OwnDrawable : public Drawable {
    virtual void Draw() {
        const double scale = 0.2;
        glPushMatrix();
        glMultMatrixd(gl_mat);
        glColor3d(color[0], color[1], color[2]);
        glBegin(GL_LINES);
            glVertex3f(0.0, 0.0, -scale);
            glVertex3f(0.0, 0.0, scale);
            glVertex3f(0.0, -scale, 0.0);
            glVertex3f(0.0, scale, 0.0);
            glVertex3f(-scale, 0.0, 0.0);
            glVertex3f(scale, 0.0, 0.0);
        glEnd();
        glPopMatrix();
    }
};
Drawable d_marker;
OwnDrawable d_points[1000];
int own_drawable_count;

bool reset=false;
void videocallback(IplImage *image)
{
    static IplImage *rgb = 0;
    static IplImage* bg_image = 0;

    bool flip_image = (image->origin?true:false);
    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }

    if (init) {
        init = false;
        sfm->Clear();
        cout<<"Loading calibration: "<<calibrationFilename.str();
        if (sfm->GetCamera()->SetCalib(calibrationFilename.str().c_str(), image->width, image->height)) {
            cout<<" [Ok]"<<endl;
        } else {
            sfm->GetCamera()->SetRes(image->width, image->height);
            cout<<" [Fail]"<<endl;
        }
        double p[16];
        sfm->GetCamera()->GetOpenglProjectionMatrix(p,image->width,image->height);
        GlutViewer::SetGlProjectionMatrix(p);
        d_marker.SetScale(marker_size*2);
        rgb = CvTestbed::Instance().CreateImageWithProto("RGB", image, 0, 3);
        CvTestbed::Instance().ToggleImageVisible(0, 0);
        bg_image = CvTestbed::Instance().CreateImage("BG texture", cvSize(512,512),8, 3);
        bg_image->origin = 0;

        sfm->SetScale(10);
        if (sfm->AddMultiMarker("mmarker.xml", FILE_FORMAT_XML)) {
            std::cout<<"Using MultiMarker defined in mmarker.xml."<<std::endl;
        } else {
            std::cout<<"Couldn't load mmarker.xml. Using default 'SampleMultiMarker' setup."<<std::endl;
            Pose pose;
            pose.Reset();
            sfm->AddMarker(0, marker_size*2, pose);
            pose.SetTranslation(-marker_size*2.5, +marker_size*1.5, 0);
            sfm->AddMarker(1, marker_size, pose);
            pose.SetTranslation(+marker_size*2.5, +marker_size*1.5, 0);
            sfm->AddMarker(2, marker_size, pose);
            pose.SetTranslation(-marker_size*2.5, -marker_size*1.5, 0);
            sfm->AddMarker(3, marker_size, pose);
            pose.SetTranslation(+marker_size*2.5, -marker_size*1.5, 0);
            sfm->AddMarker(4, marker_size, pose);
        }
        sfm->SetResetPoint();
    }
    if (reset) {
        sfm->Reset();
        reset = false;
    }

    //if (sfm->UpdateRotationsOnly(image)) {
    //if (sfm->UpdateTriangulateOnly(image)) {
    if (sfm->Update(image, false, true, 7.f, 15.f)) {
        // Draw the camera (The GlutViewer has little weirdness here...)q
        Pose pose = *(sfm->GetPose());
        double gl[16];
        pose.GetMatrixGL(gl, true);
        GlutViewer::SetGlModelviewMatrix(gl);
        pose.Invert();
        pose.GetMatrixGL(d_marker.gl_mat, false);
        GlutViewer::DrawableClear();
        GlutViewer::DrawableAdd(&d_marker);
        own_drawable_count=0;

        // Draw features
        std::map<int, SimpleSfM::Feature>::iterator iter;
        iter = sfm->container.begin();
        for(;iter != sfm->container.end(); iter++) {
			if (sfm->container_triangulated.find(iter->first) !=  sfm->container_triangulated.end()) continue;
            if (iter->second.has_p3d) 
            { 
                if (own_drawable_count < 1000) {
                    memset(d_points[own_drawable_count].gl_mat, 0, 16*sizeof(double));
                    d_points[own_drawable_count].gl_mat[0]  = 1;
                    d_points[own_drawable_count].gl_mat[5]  = 1;
                    d_points[own_drawable_count].gl_mat[10] = 1;
                    d_points[own_drawable_count].gl_mat[15] = 1;
                    d_points[own_drawable_count].gl_mat[12] = iter->second.p3d.x;
                    d_points[own_drawable_count].gl_mat[13] = iter->second.p3d.y;
                    d_points[own_drawable_count].gl_mat[14] = iter->second.p3d.z;
                    if (iter->second.type_id == 0) d_points[own_drawable_count].SetColor(1,0,0);
                    else d_points[own_drawable_count].SetColor(0,1,0);
                    GlutViewer::DrawableAdd(&(d_points[own_drawable_count]));
                    own_drawable_count++;
                }
            }
        }

		// Draw triangulated features
        iter = sfm->container_triangulated.begin();
        for(;iter != sfm->container_triangulated.end(); iter++) {
            if (iter->second.has_p3d) 
            { 
                if (own_drawable_count < 1000) {
                    memset(d_points[own_drawable_count].gl_mat, 0, 16*sizeof(double));
                    d_points[own_drawable_count].gl_mat[0]  = 1;
                    d_points[own_drawable_count].gl_mat[5]  = 1;
                    d_points[own_drawable_count].gl_mat[10] = 1;
                    d_points[own_drawable_count].gl_mat[15] = 1;
                    d_points[own_drawable_count].gl_mat[12] = iter->second.p3d.x;
                    d_points[own_drawable_count].gl_mat[13] = iter->second.p3d.y;
                    d_points[own_drawable_count].gl_mat[14] = iter->second.p3d.z;
                    /*if (iter->second.type_id == 0) d_points[own_drawable_count].SetColor(1,0,1);
                    else*/ d_points[own_drawable_count].SetColor(0,0,1);
                    GlutViewer::DrawableAdd(&(d_points[own_drawable_count]));
                    own_drawable_count++;
                }
            }
        }
    }
    if (image->nChannels == 1) cvCvtColor(image, rgb, CV_GRAY2RGB);
    else if (image->nChannels == 3) cvCopy(image, rgb);

    // Draw video on GlutViewer background
    cvResize(rgb, bg_image);
    GlutViewer::SetVideo(bg_image);

    // Draw debug info to the rgb
    sfm->Draw(rgb);

    if (flip_image) {
        cvFlip(image);
        image->origin = !image->origin;
    }
}


int keycallback(int key)
{
    if(key == 'r')
    {
        reset = true;
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
        std::cout << "SamplePointcloud" << std::endl;
        std::cout << "================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This example shows simple structure from motion approach that can be "<< std::endl;
        std::cout << "  used to track environment beyond an multimarker setup. To get this   "<< std::endl;
        std::cout << "  example work properly be sure to calibrate your camera and tune it   "<< std::endl;
        std::cout << "  to have fast framerate without motion blur.                          "<< std::endl;
        std::cout << std::endl;
        std::cout << "  There are two possible approaches Update() and UpdateRotationsOnly()."<< std::endl;
        std::cout << "  By default the Update() is used but you can easily uncomment the     "<< std::endl;
        std::cout << "  other one if needed."<< std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " [device]" << std::endl;
        std::cout << std::endl;
        std::cout << "    device    integer selecting device from enumeration list (default 0)" << std::endl;
        std::cout << "              highgui capture devices are prefered" << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  r: reset" << std::endl;
        std::cout << "  q: quit" << std::endl;
        std::cout << std::endl;

        // Initialise GlutViewer and CvTestbed
        GlutViewer::Start(argc, argv, 640, 480, 100);
        CvTestbed::Instance().SetKeyCallback(keycallback);
        CvTestbed::Instance().SetVideoCallback(videocallback);
        sfm = new SimpleSfM();

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
            title << "SamplePointcloud (" << cap->captureDevice().captureType() << ")";

            CvTestbed::Instance().StartVideo(cap, title.str().c_str());

            if (cap->saveSettings(settingsFilename.str())) {
                std::cout << "Saving settings: " << settingsFilename.str() << std::endl;
            }

            cap->stop();
            delete cap; cap = NULL;
        }
        else if (CvTestbed::Instance().StartVideo(0, argv[0])) {
        }
        else {
            std::cout << "Could not initialize the selected capture backend." << std::endl;
        }
        delete sfm; sfm = NULL;

        return 0;
    }
    catch (const std::exception &e) {
        std::cout << "Exception: " << e.what() << endl;
    }
    catch (...) {
        std::cout << "Exception: unknown" << std::endl;
    }
}

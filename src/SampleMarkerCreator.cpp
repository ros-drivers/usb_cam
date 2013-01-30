#include "MultiMarker.h"
#include "highgui.h"
using namespace std;
using namespace alvar;

struct State {
    IplImage *img;
    stringstream filename;
    double minx, miny, maxx, maxy; // top-left and bottom-right in pixel units
    MultiMarker multi_marker;

    // General options
    bool   prompt;
    double units;           // how many pixels per one unit
    double marker_side_len; // marker side len in current units
    int    marker_type;     // 0:MarkerData, 1:ArToolkit
    double posx, posy;      // The position of marker center in the given units
    int    content_res;
    double margin_res;

    // MarkerData specific options
    MarkerData::MarkerContentType marker_data_content_type;
    bool                          marker_data_force_strong_hamming;

    State() 
        : img(0),
          prompt(false),
          units(96.0/2.54),      // cm assuming 96 dpi
          marker_side_len(9.0),  // 9 cm
          marker_type(0),
          posx(0), posy(0),
          content_res(0),        // 0 uses default
          margin_res(0.0),       // 0.0 uses default (can be n*0.5)
          marker_data_content_type(MarkerData::MARKER_CONTENT_TYPE_NUMBER),
          marker_data_force_strong_hamming(false)
    {}
    ~State() {
        if (img) cvReleaseImage(&img);
    }
    void AddMarker(const char *id) {
        if (marker_type == 0) {
            MarkerData md(marker_side_len, content_res, margin_res);
            int side_len = int(marker_side_len*units+0.5);
            if (img == 0) {
                img = cvCreateImage(cvSize(side_len, side_len), IPL_DEPTH_8U, 1);
                filename.str("");
                filename<<"MarkerData";
                minx = (posx*units) - (marker_side_len*units/2.0);
                miny = (posy*units) - (marker_side_len*units/2.0);
                maxx = (posx*units) + (marker_side_len*units/2.0);
                maxy = (posy*units) + (marker_side_len*units/2.0);
            } else {
                double new_minx = (posx*units) - (marker_side_len*units/2.0);
                double new_miny = (posy*units) - (marker_side_len*units/2.0);
                double new_maxx = (posx*units) + (marker_side_len*units/2.0);
                double new_maxy = (posy*units) + (marker_side_len*units/2.0);
                if (minx < new_minx) new_minx = minx;
                if (miny < new_miny) new_miny = miny;
                if (maxx > new_maxx) new_maxx = maxx;
                if (maxy > new_maxy) new_maxy = maxy;
                IplImage *new_img = cvCreateImage(cvSize(int(new_maxx-new_minx+0.5), int(new_maxy-new_miny+0.5)), IPL_DEPTH_8U, 1);
                cvSet(new_img, cvScalar(255));
                CvRect roi = cvRect(int(minx-new_minx+0.5), int(miny-new_miny+0.5), img->width, img->height);
                cvSetImageROI(new_img, roi);
                cvCopy(img, new_img);
                cvReleaseImage(&img);
                img = new_img;
                roi.x = int((posx*units) - (marker_side_len*units/2.0) - new_minx + 0.5); 
                roi.y = int((posy*units) - (marker_side_len*units/2.0) - new_miny + 0.5); 
                roi.width = int(marker_side_len*units+0.5); roi.height = int(marker_side_len*units+0.5);
                cvSetImageROI(img, roi);
                minx = new_minx; miny = new_miny;
                maxx = new_maxx; maxy = new_maxy;
            }
            if (marker_data_content_type == MarkerData::MARKER_CONTENT_TYPE_NUMBER) {
                int idi = atoi(id);
                md.SetContent(marker_data_content_type, idi, 0);
                if (filename.str().length()<64) filename<<"_"<<idi;

                Pose pose;
                pose.Reset();
                pose.SetTranslation(posx, -posy, 0);
                multi_marker.PointCloudAdd(idi, marker_side_len, pose);
            } else {
                md.SetContent(marker_data_content_type, 0, id);
                const char *p = id;
                int counter=0;
                filename<<"_";
                while(*p) {
                    if (!isalnum(*p)) filename<<"_";
                    else filename<<(char)tolower(*p);
                    p++; counter++;
                    if (counter > 8) break;
                }
            }
            md.ScaleMarkerToImage(img);
            cvResetImageROI(img);
        }
        else if (marker_type == 1) {
            // Create and save MarkerArtoolkit marker (Note, that this doesn't support multi markers)
            MarkerArtoolkit md(marker_side_len, content_res, margin_res);
            int side_len = int(marker_side_len*units+0.5);
            if (img != 0) cvReleaseImage(&img);
            img = cvCreateImage(cvSize(side_len, side_len), IPL_DEPTH_8U, 1);
            filename.str("");
            filename<<"MarkerArtoolkit";
            md.SetContent(atoi(id));
            filename<<"_"<<atoi(id);
            md.ScaleMarkerToImage(img);
        }
    }
    void Save() {
        if (img) {
            std::stringstream filenamexml;
            filenamexml<<filename.str()<<".xml";
            filename<<".png";
            std::cout<<"Saving: "<<filename.str()<<std::endl;
            cvSaveImage(filename.str().c_str(), img);
            if (multi_marker.Size() > 1) {
                std::cout<<"Saving: "<<filenamexml.str()<<std::endl;
                multi_marker.Save(filenamexml.str().c_str(), alvar::FILE_FORMAT_XML);
            }
        }
    }
} st;

int main(int argc, char *argv[])
{
    try {
        if (argc < 2) st.prompt = true;
        for (int i=1; i<argc; i++) {
            if (strcmp(argv[i],"-f") == 0) 
                st.marker_data_force_strong_hamming=true;
            else if (strcmp(argv[i],"-1") == 0) 
                st.marker_data_content_type = MarkerData::MARKER_CONTENT_TYPE_STRING;
            else if (strcmp(argv[i],"-2") == 0) 
                st.marker_data_content_type = MarkerData::MARKER_CONTENT_TYPE_FILE;
            else if (strcmp(argv[i],"-3") == 0) 
                st.marker_data_content_type = MarkerData::MARKER_CONTENT_TYPE_HTTP;
            else if (strcmp(argv[i],"-u") == 0)
                st.units = atof(argv[++i]);
            else if (strcmp(argv[i],"-uin") == 0)
                st.units = (96.0);
            else if (strcmp(argv[i],"-ucm") == 0)
                st.units = (96.0/2.54);
            else if (strcmp(argv[i],"-s") == 0)
                st.marker_side_len = atof(argv[++i]);
            else if (strcmp(argv[i],"-r") == 0)
                st.content_res = atoi(argv[++i]);
            else if (strcmp(argv[i],"-m") == 0)
                st.margin_res = atof(argv[++i]);
            else if (strcmp(argv[i],"-a") == 0)
                st.marker_type = 1;
            else if (strcmp(argv[i],"-p") == 0)
                st.prompt = true;
            else if (strcmp(argv[i],"-xy") == 0) {
                st.posx = atof(argv[++i]);
                st.posy = atof(argv[++i]);
            }
            else st.AddMarker(argv[i]);
        }

        // Output usage message
        if (st.prompt) {
            std::string filename(argv[0]);
            filename = filename.substr(filename.find_last_of('\\') + 1);
            std::cout << "SampleMarkerCreator" << std::endl;
            std::cout << "===================" << std::endl;
            std::cout << std::endl;
            std::cout << "Description:" << std::endl;
            std::cout << "  This is an example of how to use the 'MarkerData' and 'MarkerArtoolkit'" << std::endl;
            std::cout << "  classes to generate marker images. This application can be used to" << std::endl;
            std::cout << "  generate markers and multimarker setups that can be used with" << std::endl;
            std::cout << "  SampleMarkerDetector and SampleMultiMarker." << std::endl;
            std::cout << std::endl;
            std::cout << "Usage:" << std::endl;
            std::cout << "  " << filename << " [options] argument" << std::endl;
            std::cout << std::endl;
            std::cout << "    65535             marker with number 65535" << std::endl;
            std::cout << "    -f 65535          force hamming(8,4) encoding" << std::endl;
            std::cout << "    -1 \"hello world\"  marker with string" << std::endl;
            std::cout << "    -2 catalog.xml    marker with file reference" << std::endl;
            std::cout << "    -3 www.vtt.fi     marker with URL" << std::endl;
            std::cout << "    -u 96             use units corresponding to 1.0 unit per 96 pixels" << std::endl;
            std::cout << "    -uin              use inches as units (assuming 96 dpi)" << std::endl;
            std::cout << "    -ucm              use cm's as units (assuming 96 dpi) <default>" << std::endl;
            std::cout << "    -s 5.0            use marker size 5.0x5.0 units (default 9.0x9.0)" << std::endl;
            std::cout << "    -r 5              marker content resolution -- 0 uses default" << std::endl;
            std::cout << "    -m 2.0            marker margin resolution -- 0 uses default" << std::endl;
            std::cout << "    -a                use ArToolkit style matrix markers" << std::endl;
            std::cout << "    -p                prompt marker placements interactively from the user" << std::endl;
            std::cout << std::endl;

            // Interactive stuff here
            if (st.prompt) {
                st.marker_type = 0;
                st.marker_data_content_type = MarkerData::MARKER_CONTENT_TYPE_NUMBER;
                std::cout<<"\nPrompt marker placements interactively"<<std::endl;
                std::cout<<"  units: "<<st.units/96.0*2.54<<" cm "<<st.units/96.0<<" inches"<<std::endl;
                std::cout<<"  marker side: "<<st.marker_side_len<<" units"<<std::endl;
                bool loop=true;
                std::string s;
                int marker_id=0;
                double posx=0.0, posy=0.0;
                bool vert=false;
                while(loop) {
                    std::cout<<"  marker id (use -1 to end) ["<<marker_id<<"]: "; std::flush(std::cout);
                    std::getline(std::cin, s); if (s.length() > 0) marker_id=atoi(s.c_str());
                    if (marker_id < 0) break;
                    std::cout<<"  x position (in current units) ["<<posx<<"]: "; std::flush(std::cout);
                    std::getline(std::cin, s); if (s.length() > 0) posx=atof(s.c_str());
                    std::cout<<"  y position (in current units) ["<<posy<<"]: "; std::flush(std::cout);
                    std::getline(std::cin, s); if (s.length() > 0) posy=atof(s.c_str());
                    st.posx=posx; st.posy=posy;
                    std::stringstream ss;
                    ss<<marker_id;
                    st.AddMarker(ss.str().c_str());

                    // Guess suitable marker_id and placement for the next marker
                    marker_id++;
                    if (posx <= 0) {
                        posx = int(sqrt(double(marker_id)))*st.marker_side_len*2;
                        posy = 0;
                        vert = true;
                    } else if (vert) {
                        posy += (st.marker_side_len*2);
                        if (posy >= posx) vert = false;
                    } else {
                        posx -= (st.marker_side_len*2);
                    }
                }
            }
        }
        st.Save();
        return 0;
    }
    catch (const std::exception &e) {
        std::cout << "Exception: " << e.what() << endl;
    }
    catch (...) {
        std::cout << "Exception: unknown" << std::endl;
    }
}

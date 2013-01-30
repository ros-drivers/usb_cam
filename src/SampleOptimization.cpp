#include "Optimization.h"
#include "cv.h"
#include "highgui.h"
#include <time.h>
#include <vector>
#include <iostream>
#include <string>
using namespace std;
using namespace alvar;

const int res=640;
const double poly_res=8.0;

double random(int dist_type, double param1, double param2) {
    static CvRNG rng=0;
    if (rng == 0) rng = cvRNG(time(0));
    double m_data;
    CvMat m = cvMat(1, 1, CV_64F, &m_data);
    cvRandArr(&rng, &m, dist_type, cvScalar(param1), cvScalar(param2));
    return m_data;
}

double get_y(double x, double a, double b, double c, double d, double e) {
    return (a*x*x*x*x + b*x*x*x + c*x*x + d*x + e);
}

// Just generate some random data that can be used as sensor input
bool get_measurement(double *x, double *y, double a, double b, double c, double d, double e) {
    double xx = random(CV_RAND_UNI, -(poly_res/2), +(poly_res/2)); //(rand()*poly_res/RAND_MAX)-(poly_res/2);
    double yy = get_y(xx, a, b, c, d, e);
    double ry = random(CV_RAND_NORMAL, 0, poly_res/8); //(rand()*(poly_res/4)/RAND_MAX)-(poly_res/8);
    yy += ry;
    *x = xx;
    *y = yy;
    if (*y < -(poly_res/2)) return false;
    if (*y >= (poly_res/2)) return false;
    return true;
}

void Estimate(CvMat* state, CvMat *projection, void *param) {
    double *measx=(double *)param;
    int data_degree = state->rows-1;
    double a = (data_degree >= 4? cvmGet(state, 4, 0) : 0);
    double b = (data_degree >= 3? cvmGet(state, 3, 0) : 0);
    double c = (data_degree >= 2? cvmGet(state, 2, 0) : 0);
    double d = (data_degree >= 1? cvmGet(state, 1, 0) : 0);
    double e = (data_degree >= 0? cvmGet(state, 0, 0) : 0);
    for (int i=0; i<projection->rows; i++) {
        cvmSet(projection, i, 0, get_y(measx[i], a, b, c, d, e));
    }
}

int main(int argc, char *argv[])
{
    try {
        // Output usage message
        std::string filename(argv[0]);
        filename = filename.substr(filename.find_last_of('\\') + 1);
        std::cout << "SampleOptimization" << std::endl;
        std::cout << "==================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'Optimization' class. Random data" << std::endl;
        std::cout << "  is generated and approximated using curves of increasing degrees." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  any key: cycle through datasets" << std::endl;
        std::cout << "  q: quit" << std::endl;
        std::cout << std::endl;

        // Processing loop
        IplImage *img = cvCreateImage(cvSize(res,res), IPL_DEPTH_8U, 3);
        cvNamedWindow("SampleOptimization");
        for (int data_degree=0; data_degree<5; data_degree++) {
            double a = (data_degree >= 4? random(CV_RAND_UNI, -0.5, 0.5) : 0);
            double b = (data_degree >= 3? random(CV_RAND_UNI, -0.5, 0.5) : 0);
            double c = (data_degree >= 2? random(CV_RAND_UNI, -0.5, 0.5) : 0);
            double d = (data_degree >= 1? random(CV_RAND_UNI, -0.5, 0.5) : 0);
            double e = (data_degree >= 0? random(CV_RAND_UNI, -0.5, 0.5) : 0);
            cvZero(img);
            vector<CvPoint2D32f> measvec;
            for (int i=0; i<1000; i++) {
                double x, y;
                if (get_measurement(&x, &y, a, b, c, d, e)) {
                    measvec.push_back(cvPoint2D32f(x, y));
                    x = (x*res/poly_res)+(res/2);
                    y = (y*res/poly_res)+(res/2);
                    cvCircle(img, cvPoint(int(x), int(y)), 1, CV_RGB(0,255,0));
                }
            }
            cvShowImage("SampleOptimization", img);
            cvWaitKey(10);
            double measx[1000];
            CvMat *meas = cvCreateMat(measvec.size(), 1, CV_64F);
            for (size_t i=0; i<measvec.size(); i++) {
                measx[i] = measvec[i].x;
                cvmSet(meas, i, 0, measvec[i].y);
            }
            for (int degree=0; degree<5; degree++) 
            {
                double param_data[5]={0};
                CvMat param = cvMat(degree+1, 1, CV_64F, param_data);
                Optimization opt(param.rows, meas->rows);
                opt.Optimize(&param, meas, 0.1, 100, Estimate, measx);
                double a = (degree >= 4? cvmGet(&param, 4, 0) : 0);
                double b = (degree >= 3? cvmGet(&param, 3, 0) : 0);
                double c = (degree >= 2? cvmGet(&param, 2, 0) : 0);
                double d = (degree >= 1? cvmGet(&param, 1, 0) : 0);
                double e = (degree >= 0? cvmGet(&param, 0, 0) : 0);
                const int step=5;
                for (int x2=step; x2<res; x2+=step) {
                    int x1 = x2-step;
                    double xx1 = (x1*poly_res/res)-(poly_res/2);
                    double xx2 = (x2*poly_res/res)-(poly_res/2);
                    double yy1 = get_y(xx1, a, b, c, d, e);
                    double yy2 = get_y(xx2, a, b, c, d, e);
                    int y1 = int((yy1*res/poly_res)+(res/2));
                    int y2 = int((yy2*res/poly_res)+(res/2));
                    cvLine(img, cvPoint(x1,y1), cvPoint(x2,y2), CV_RGB(degree*50,255-(degree*50),255));
                }
                cvShowImage("SampleOptimization", img);
                cvWaitKey(10);
            }
            cvReleaseMat(&meas);
            cvShowImage("SampleOptimization", img);
            int key = cvWaitKey(0);
            if (key == 'q') {
                break;
            }
        }
        cvReleaseImage(&img);
        return 0;
    }
    catch (const std::exception &e) {
        std::cout << "Exception: " << e.what() << endl;
    }
    catch (...) {
        std::cout << "Exception: unknown" << std::endl;
    }
}

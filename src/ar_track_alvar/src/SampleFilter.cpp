#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <cstdlib>
#include <string>
#include "Filter.h"
#include "Kalman.h"
#include "Platform.h"
#include "AlvarException.h"
using namespace alvar;
using namespace std;

const int res=320;

void filter_none(double x, double y, double *fx, double *fy) {
    *fx = x; *fy = y;
}

void filter_average(double x, double y, double *fx, double *fy) {
    static FilterAverage ax(30);
    static FilterAverage ay(30);
    *fx = ax.next(x);
    *fy = ay.next(y);
}

void filter_median(double x, double y, double *fx, double *fy) {
    static FilterMedian ax(30);
    static FilterMedian ay(30);
    *fx = ax.next(x);
    *fy = ay.next(y);
}

void filter_running_average(double x, double y, double *fx, double *fy) {
    static FilterRunningAverage ax(0.03);
    static FilterRunningAverage ay(0.03);
    *fx = ax.next(x);
    *fy = ay.next(y);
}

void filter_des(double x, double y, double *fx, double *fy) {
    static FilterDoubleExponentialSmoothing ax(0.03,0.01);
    static FilterDoubleExponentialSmoothing ay(0.03,0.01);
    *fx = ax.next(x);
    *fy = ay.next(y);
}

void filter_kalman(double x, double y, double *fx, double *fy) {
    static bool init=true;
    static KalmanSensor sensor(4,2); 
    static Kalman kalman(4); // x, y, dx, dy
    if (init) {
        init = false;
        // H
        cvZero(sensor.H);
        cvmSet(sensor.H, 0, 0, 1);
        cvmSet(sensor.H, 1, 1, 1);
        // R
        cvSetIdentity(sensor.R, cvScalar(10));
        // F
        cvSetIdentity(kalman.F);
        cvmSet(kalman.F, 0, 2, 1);
        cvmSet(kalman.F, 1, 3, 1);
        // Q
        cvmSet(kalman.Q, 0, 0, 0.0001);
        cvmSet(kalman.Q, 1, 1, 0.0001);
        cvmSet(kalman.Q, 2, 2, 0.000001);
        cvmSet(kalman.Q, 3, 3, 0.000001);
        // P
        cvSetIdentity(kalman.P, cvScalar(100));
    }
    cvmSet(sensor.z, 0, 0, x);
    cvmSet(sensor.z, 1, 0, y);
    kalman.predict_update(&sensor, (unsigned long)(cv::getTickCount() / cv::getTickFrequency() * 1000));
    *fx = cvmGet(kalman.x, 0, 0);
    *fy = cvmGet(kalman.x, 1, 0);
}

void filter_array_average(double x, double y, double *fx, double *fy) {
    static bool init=true;
    static FilterArray<FilterAverage> fa(2);
    if (init) {
        init=false;
        for (int i=0; i<2; i++) {
            fa[i].setWindowSize(30);
        }
    }
    *fx = fa[0].next(x);
    *fy = fa[1].next(y);
}

class KalmanSensorOwn : public KalmanSensorEkf {
    virtual void h(CvMat *x_pred, CvMat *_z_pred) {
        double x = cvmGet(x_pred, 0, 0);
        double y = cvmGet(x_pred, 1, 0);
        double dx = cvmGet(x_pred, 2, 0);
        double dy = cvmGet(x_pred, 3, 0);
        cvmSet(_z_pred, 0, 0, x);
        cvmSet(_z_pred, 1, 0, y);
    }
public:
    KalmanSensorOwn(int _n, int _m) : KalmanSensorEkf(_n, _m) {}
};

class KalmanOwn : public KalmanEkf {
    virtual void f(CvMat *_x, CvMat *_x_pred, double dt) {
        double x = cvmGet(_x, 0, 0);
        double y = cvmGet(_x, 1, 0);
        double dx = cvmGet(_x, 2, 0);
        double dy = cvmGet(_x, 3, 0);
        cvmSet(_x_pred, 0, 0, x + dt*dx);
        cvmSet(_x_pred, 1, 0, y + dt*dy);
        cvmSet(_x_pred, 2, 0, dx);
        cvmSet(_x_pred, 3, 0, dy);
    }
public:
    KalmanOwn(int _n) : KalmanEkf(_n) {}
};

void filter_ekf(double x, double y, double *fx, double *fy) {
    static bool init=true;
    static KalmanSensorOwn sensor(4,2); 
    static KalmanOwn kalman(4); // x, y, dx, dy
    if (init) {
        init = false;
        // R
        cvSetIdentity(sensor.R, cvScalar(100));
        // Q
        cvmSet(kalman.Q, 0, 0, 0.001);
        cvmSet(kalman.Q, 1, 1, 0.001);
        cvmSet(kalman.Q, 2, 2, 0.01);
        cvmSet(kalman.Q, 3, 3, 0.01);
        // P
        cvSetIdentity(kalman.P, cvScalar(100));
    }
    cvmSet(sensor.z, 0, 0, x);
    cvmSet(sensor.z, 1, 0, y);
    kalman.predict_update(&sensor, (unsigned long)(cv::getTickCount() / cv::getTickFrequency() * 1000));
    *fx = cvmGet(kalman.x, 0, 0);
    *fy = cvmGet(kalman.x, 1, 0);
}

//Make list of filters
const int nof_filters = 8;
void (*(filters[nof_filters]))(double x, double y, double *fx, double *fy) = {
    filter_none,
    filter_average,
    filter_median,
    filter_running_average,
    filter_des,
    filter_kalman,
    filter_ekf,
    filter_array_average,
};
char filter_names[nof_filters][64]={
    "No filter - Press any key to change",
    "Average",
    "Median",
    "Running Average",
    "Double Exponential Smoothing",
    "Kalman",
    "Extended Kalman",
    "Array (average)"
};

// Just generate some random data that can be used as sensor input
void get_measurement(double *x, double *y) {
    static double xx=0;
    static double yy=0;
    static double dxx = 0.3;
    static double dyy = 0.7;
    xx += dxx; yy += dyy;
    if ((xx > res) || (xx < 0)) dxx = -dxx;
    if ((yy > res) || (yy < 0)) dyy = -dyy;
    double rx = (rand()*20.0/RAND_MAX)-10.0;
    double ry = (rand()*20.0/RAND_MAX)-10.0;

    // Add some outliers
    if(fabs(rx*ry)>50)
    {
        rx *= 5;
        ry *= 5;
    }

    *x = xx + rx; *y = yy + ry;
}

int main(int argc, char *argv[])
{
    try {
        // Output usage message
        std::string filename(argv[0]);
        filename = filename.substr(filename.find_last_of('\\') + 1);
        std::cout << "SampleFilter" << std::endl;
        std::cout << "============" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'FilterAverage', 'FilterMedian'," << std::endl;
        std::cout << "  'FilterRunningAverage', 'FilterDoubleExponentialSmoothing', 'Kalman'" << std::endl;
        std::cout << "  'KalmanEkf' and 'FilterArray' filtering classes. First the example" << std::endl;
        std::cout << "  shows unfiltered test data with outliers. The data is then filtered" << std::endl;
        std::cout << "  using the various filters. Press any key to cycle through the filters." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << std::endl;
        std::cout << std::endl;
        std::cout << "Keyboard Shortcuts:" << std::endl;
        std::cout << "  any key: cycle through filters" << std::endl;
        std::cout << "  q: quit" << std::endl;
        std::cout << std::endl;

        // Processing loop
        IplImage *img = cvCreateImage(cvSize(res, res), IPL_DEPTH_8U, 3);
        cvNamedWindow("SampleFilter");
        CvFont font;
        cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0);
        for (int ii = 0; ii < nof_filters; ii++) {
            int key = 0;
            double x, y;
            double fx, fy;
            vector<CvPoint> tail;
            while (1) {
                get_measurement(&x, &y);
                filters[ii](x, y, &fx, &fy);
                cvZero(img);
                cvPutText(img, filter_names[ii], cvPoint(3, res - 10), &font, CV_RGB(255, 255, 255));
                cvCircle(img, cvPoint(int(x), int(y)), 2, CV_RGB(0, 255, 255));
                cvCircle(img, cvPoint(int(x), int(y)), 3, CV_RGB(255, 255, 255));
                CvPoint fp;
                fp.x = int(fx);
                fp.y = int(fy);
                tail.push_back(fp);
                for (size_t iii = 0; iii < tail.size(); iii++) {
                    cvCircle(img, tail[iii], 0, CV_RGB(255, 255, 0));
                }
                cvCircle(img, fp, 2, CV_RGB(255, 0, 255));
                cvShowImage("SampleFilter", img);
                key = cvWaitKey(10);
                if (key != -1) {
                    break;
                }
            }
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

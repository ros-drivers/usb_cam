#ifndef SHARED_H
#define SHARED_H

#include "CaptureFactory.h"

using namespace alvar;

void outputEnumeratedPlugins(CaptureFactory::CapturePluginVector &plugins)
{
    for (int i = 0; i < (int)plugins.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << plugins.at(i);
    }

    std::cout << std::endl;
}

void outputEnumeratedDevices(CaptureFactory::CaptureDeviceVector &devices, int selectedDevice)
{
    for (int i = 0; i < (int)devices.size(); ++i) {
        if (selectedDevice == i) {
            std::cout << "* ";
        }
        else {
            std::cout << "  ";
        }

        std::cout << i << ": " << devices.at(i).uniqueName();

        if (devices[i].description().length() > 0) {
            std::cout << ", " << devices.at(i).description();
        }

        std::cout << std::endl;
    }
}

int defaultDevice(CaptureFactory::CaptureDeviceVector &devices)
{
    for (int i = 0; i < (int)devices.size(); ++i) {
        if (devices.at(i).captureType() == "highgui") {
            return i;
        }
    }

    return 0;
}

#endif

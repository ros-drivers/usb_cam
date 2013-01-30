/*
 * This file is part of ALVAR, A Library for Virtual and Augmented Reality.
 *
 * Copyright 2007-2012 VTT Technical Research Centre of Finland
 *
 * Contact: VTT Augmented Reality Team <alvar.info@vtt.fi>
 *          <http://www.vtt.fi/multimedia/alvar.html>
 *
 * ALVAR is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ALVAR; if not, see
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>.
 */

#include "CapturePluginPtgrey.h"
#include <../../FlyCapture2/include/Camera.h>
#include <../../FlyCapture2/include/Image.h>
#include <../../FlyCapture2/include/Error.h>
#include <../../FlyCapture2/include/BusManager.h>

#include <sstream>

using namespace std;

namespace alvar {
namespace plugins {

CapturePtgrey::CapturePtgrey(const CaptureDevice captureDevice)
    : Capture(captureDevice)
    , mCamera(new FlyCapture2::Camera)
    , mImage(new FlyCapture2::Image)
    , mChannels(-1)
    , mReturnFrame(NULL)
{
}

CapturePtgrey::~CapturePtgrey()
{
    stop();
    delete mCamera;
    delete mImage;
}

bool CapturePtgrey::start()
{
    if (isCapturing()) {
        return isCapturing();
    }
    
    stringstream id(captureDevice().id());
    id.setf(ios_base::hex, ios_base::basefield);
    id >> mGUID.value[0]; id.get();
    id >> mGUID.value[1]; id.get();
    id >> mGUID.value[2]; id.get();
    id >> mGUID.value[3];

    if (mCamera->Connect(&mGUID) != FlyCapture2::PGRERROR_OK) {
        return false;
    }

    FlyCapture2::VideoMode videoMode;
    FlyCapture2::FrameRate frameRate;
    if (mCamera->GetVideoModeAndFrameRate (&videoMode, &frameRate) != FlyCapture2::PGRERROR_OK) {
        return false;
    }
    
    if (videoMode == FlyCapture2::VIDEOMODE_640x480RGB) {
      mChannels = 3;
      mXResolution = 640;
      mYResolution = 480;
    
    } else if (videoMode == FlyCapture2::VIDEOMODE_640x480Y8) {
      mChannels = 1;
      mXResolution = 640;
      mYResolution = 480;
    
    } else if (videoMode == FlyCapture2::VIDEOMODE_800x600RGB) {
      mChannels = 3;
      mXResolution = 800;
      mYResolution = 600;
    
    } else if (videoMode == FlyCapture2::VIDEOMODE_800x600Y8) {
      mChannels = 1;
      mXResolution = 800;
      mYResolution = 600;
    
    } else if (videoMode == FlyCapture2::VIDEOMODE_1024x768RGB) {
      mChannels = 3;
      mXResolution = 1024;
      mYResolution = 768;
    
    } else if (videoMode == FlyCapture2::VIDEOMODE_1024x768Y8) {
      mChannels = 1;
      mXResolution = 1024;
      mYResolution = 768;
    
    } else if (videoMode == FlyCapture2::VIDEOMODE_1280x960RGB) {
      mChannels = 3;
      mXResolution = 1280;
      mYResolution = 960;
    
    } else if (videoMode == FlyCapture2::VIDEOMODE_1280x960Y8) {
      mChannels = 1;
      mXResolution = 1280;
      mYResolution = 960;
    
    } else if (videoMode == FlyCapture2::VIDEOMODE_1600x1200RGB) {
      mChannels = 3;
      mXResolution = 1600;
      mYResolution = 1200;
    
    } else if (videoMode == FlyCapture2::VIDEOMODE_1600x1200Y8) {
      mChannels = 1;
      mXResolution = 1600;
      mYResolution = 1200;
    
    } else {
        return false;
    }

    mReturnFrame = cvCreateImage(cvSize(mXResolution, mYResolution), IPL_DEPTH_8U, mChannels);
    if (mCamera->StartCapture() != FlyCapture2::PGRERROR_OK) {
        return false;
    }
    mIsCapturing = true;
    return isCapturing();
}

void CapturePtgrey::stop()
{
    if (isCapturing()) {
        mCamera->StopCapture();
        cvReleaseImage(&mReturnFrame);
    }
}

IplImage *CapturePtgrey::captureImage()
{
    if (!isCapturing()) {
        return NULL;
    }

    if (mCamera->RetrieveBuffer(mImage) == FlyCapture2::PGRERROR_OK) {
        unsigned long length = mReturnFrame->widthStep * mYResolution;
        memcpy(mReturnFrame->imageData, mImage->GetData(), length);
    }
    return mReturnFrame;
}

bool CapturePtgrey::showSettingsDialog()
{
    return false;
}

string CapturePtgrey::SerializeId()
{
    return "CapturePtgrey";
}

bool CapturePtgrey::Serialize(Serialization *serialization)
{
    return false;
}

CapturePluginPtgrey::CapturePluginPtgrey(const string &captureType)
    : CapturePlugin(captureType)
{
}

CapturePluginPtgrey::~CapturePluginPtgrey()
{
}

CapturePlugin::CaptureDeviceVector CapturePluginPtgrey::enumerateDevices()
{
    CaptureDeviceVector devices;

    FlyCapture2::BusManager bus;
    if (bus.RescanBus() != FlyCapture2::PGRERROR_OK) {
        return devices;
    }

    unsigned int numberCameras = 0;
    bus.GetNumOfCameras(&numberCameras);
    
    for (unsigned int i = 0; i < numberCameras; i++) {
        FlyCapture2::PGRGuid guid;
        bus.GetCameraFromIndex(i, &guid);
        stringstream convert;
        convert << hex << guid.value[0];
        convert << "_" << hex << guid.value[1];
        convert << "_" << hex << guid.value[2];
        convert << "_" << hex << guid.value[3];
        stringstream description;
        FlyCapture2::Camera camera;
        if (camera.Connect(&guid) != FlyCapture2::PGRERROR_OK) continue;
        FlyCapture2::CameraInfo info;
        if (camera.GetCameraInfo (&info) != FlyCapture2::PGRERROR_OK) continue;
        description << info.vendorName << " ";
        description << info.modelName;
        CaptureDevice captureDevice(mCaptureType, convert.str(), description.str());
        devices.push_back(captureDevice);
    }

    return devices;
}

Capture *CapturePluginPtgrey::createCapture(const CaptureDevice captureDevice)
{
    return new CapturePtgrey(captureDevice);
}

void registerPlugin(const string &captureType, alvar::CapturePlugin *&capturePlugin)
{
    capturePlugin = new CapturePluginPtgrey(captureType);
}

} // namespace plugins
} // namespace alvar

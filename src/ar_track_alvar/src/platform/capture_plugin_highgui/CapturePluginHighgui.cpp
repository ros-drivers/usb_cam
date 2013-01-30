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

#include "CapturePluginHighgui.h"

#include <sstream>

namespace alvar {
namespace plugins {

CaptureHighgui::CaptureHighgui(const CaptureDevice captureDevice)
    : Capture(captureDevice)
    , mVideoCapture()
    , mMatrix()
    , mImage()
{
}

CaptureHighgui::~CaptureHighgui()
{
    stop();
}

void CaptureHighgui::setResolution(const unsigned long xResolution, const unsigned long yResolution)
{
    if (mVideoCapture.isOpened()) {
        mVideoCapture.set(CV_CAP_PROP_FRAME_WIDTH, xResolution);
        mVideoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, yResolution);
        mXResolution = (int)mVideoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
        mYResolution = (int)mVideoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
    }
}

bool CaptureHighgui::start()
{
    if (isCapturing()) {
        return isCapturing();
    }

    std::istringstream convert(captureDevice().id());
    int id;
    convert >> id;

    mVideoCapture.open(id);
    if (mVideoCapture.isOpened()) {
        mIsCapturing = true;
    }

    return isCapturing();
}

void CaptureHighgui::stop()
{
    if (isCapturing()) {
        mVideoCapture.release();
        mIsCapturing = false;
    }
}

IplImage *CaptureHighgui::captureImage()
{
    if (!isCapturing()) {
        return NULL;
    }
    if (!mVideoCapture.grab()) {
        return NULL;
    }
    mVideoCapture.retrieve(mMatrix);
    mImage = mMatrix;
    return &mImage;
}

bool CaptureHighgui::showSettingsDialog()
{
    // TODO: implement this method
    return false;
}

std::string CaptureHighgui::SerializeId()
{
    return "CaptureHighgui";
}

bool CaptureHighgui::Serialize(Serialization *serialization)
{
    return false;
}

CapturePluginHighgui::CapturePluginHighgui(const std::string &captureType)
    : CapturePlugin(captureType)
{
}

CapturePluginHighgui::~CapturePluginHighgui()
{
}

CapturePlugin::CaptureDeviceVector CapturePluginHighgui::enumerateDevices()
{
    CaptureDeviceVector devices;

    bool loop = true;
    int id = 0;
    cv::VideoCapture videoCapture;
    
    while (loop) {
        std::stringstream convert;
        convert << id;
        CaptureDevice captureDevice(mCaptureType, convert.str());
        
        videoCapture.open(id);
        if (videoCapture.isOpened()) {
            int width = (int)videoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
            int height = (int)videoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
            if (width > 0 && height > 0) {
                devices.push_back(captureDevice);
            }
            else {
                loop = false;
            }
        }
        else {
            loop = false;
        }

        id++;
    }
    videoCapture.release();

    return devices;
}

Capture *CapturePluginHighgui::createCapture(const CaptureDevice captureDevice)
{
    return new CaptureHighgui(captureDevice);
}

void registerPlugin(const std::string &captureType, alvar::CapturePlugin *&capturePlugin)
{
    capturePlugin = new CapturePluginHighgui(captureType);
}

} // namespace plugins
} // namespace alvar

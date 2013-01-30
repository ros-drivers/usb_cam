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

#include "CapturePluginFile.h"

namespace alvar {
namespace plugins {

CaptureFile::CaptureFile(const CaptureDevice captureDevice)
    : Capture(captureDevice)
    , mVideoCapture()
    , mMatrix()
    , mImage()
{
}

CaptureFile::~CaptureFile()
{
    stop();
}

bool CaptureFile::start()
{
    if (isCapturing()) {
        return isCapturing();
    }

    mVideoCapture.open(captureDevice().id().c_str());
    if (mVideoCapture.isOpened()) {
        mXResolution = (int)mVideoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
        mYResolution = (int)mVideoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
        mIsCapturing = true;
    }

    return isCapturing();
}

void CaptureFile::stop()
{
    if (isCapturing()) {
        mVideoCapture.release();
        mIsCapturing = false;
    }
}

IplImage *CaptureFile::captureImage()
{
    if (!isCapturing()) {
        return NULL;
    }

    if (!mVideoCapture.grab()) {
        // try to restart the capturing when end of file is reached
        mVideoCapture.release();
        mVideoCapture.open(captureDevice().id().c_str());
        if (!mVideoCapture.isOpened()) {
            mIsCapturing = false;
            return NULL;
        }
        if (!mVideoCapture.grab()) {
            return NULL;
        }
    }
    mVideoCapture.retrieve(mMatrix);
    mImage = mMatrix;
    return &mImage;
}

bool CaptureFile::showSettingsDialog()
{
    // TODO: implement this method
    return false;
}

std::string CaptureFile::SerializeId()
{
    return "CaptureFile";
}

bool CaptureFile::Serialize(Serialization *serialization)
{
    return false;
}

CapturePluginFile::CapturePluginFile(const std::string &captureType)
    : CapturePlugin(captureType)
{
}

CapturePluginFile::~CapturePluginFile()
{
}

CapturePlugin::CaptureDeviceVector CapturePluginFile::enumerateDevices()
{
    CaptureDeviceVector devices;
    return devices;
}

Capture *CapturePluginFile::createCapture(const CaptureDevice captureDevice)
{
    return new CaptureFile(captureDevice);
}

void registerPlugin(const std::string &captureType, alvar::CapturePlugin *&capturePlugin)
{
    capturePlugin = new CapturePluginFile(captureType);
}

} // namespace plugins
} // namespace alvar

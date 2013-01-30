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

#include "CapturePluginCmu.h"

#include <sstream>

namespace alvar {
namespace plugins {

CaptureCmu::CaptureCmu(const CaptureDevice captureDevice)
    : Capture(captureDevice)
    , mCamera(new C1394Camera())
    , mChannels(-1)
    , mReturnFrame(NULL)
{
}

CaptureCmu::~CaptureCmu()
{
    stop();
    delete mCamera;
}

bool CaptureCmu::start()
{
    if (isCapturing()) {
        return isCapturing();
    }

    if (mCamera->CheckLink() != CAM_SUCCESS) {
        return false;
    }

    int numberCameras = mCamera->GetNumberCameras();
    bool cameraSelected = false;
    for (int i = 0; i < numberCameras; i++) {
        mCamera->SelectCamera(i);
        LARGE_INTEGER uniqueId;
        mCamera->GetCameraUniqueID(&uniqueId);
        std::stringstream convert;
        convert << std::hex << uniqueId.HighPart << uniqueId.LowPart;
        if (captureDevice().id().compare(convert.str()) == 0) {
            cameraSelected = true;
            break;
        }
    }

    if (!cameraSelected) {
        return false;
    }

    if (mCamera->InitCamera(true) != CAM_SUCCESS) {
        return false;
    }

    // TODO: this needs to be parameterized somehow
    if (mCamera->HasVideoMode(2, 4)) { // 1600x1200rgb
        if (mCamera->SetVideoFormat(2) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(4) != CAM_SUCCESS) return false;
        mChannels = 3;
    }
    else if (mCamera->HasVideoMode(2, 1)) { // 1280x960rgb
        if (mCamera->SetVideoFormat(2) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(1) != CAM_SUCCESS) return false;
        mChannels = 3;
    }
    else if (mCamera->HasVideoMode(1, 4)) { // 1024x768rgb
        if (mCamera->SetVideoFormat(1) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(4) != CAM_SUCCESS) return false;
        mChannels = 3;
    }
    else if (mCamera->HasVideoMode(1, 1)) { // 800x600rgb
        if (mCamera->SetVideoFormat(1) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(1) != CAM_SUCCESS) return false;
        mChannels = 3;
    }
    else if (mCamera->HasVideoMode(0, 4)) { // 640x480rgb
        if (mCamera->SetVideoFormat(0) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(4) != CAM_SUCCESS) return false;
        mChannels = 3;
    /* // TODO: also support YUV422
    }
    else if (mCamera->HasVideoMode(2, 3)) { // 1600x1200yuv422
        if (mCamera->SetVideoFormat(2) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(3) != CAM_SUCCESS) return false;
        mChannels = 2;
    }
    else if (mCamera->HasVideoMode(2, 0)) { // 1280x960yuv422
        if (mCamera->SetVideoFormat(2) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(0) != CAM_SUCCESS) return false;
        mChannels = 2;
    }
    else if (mCamera->HasVideoMode(1, 3)) { // 1024x768yuv422
        if (mCamera->SetVideoFormat(1) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(3) != CAM_SUCCESS) return false;
        mChannels = 2;
    }
    else if (mCamera->HasVideoMode(1, 0)) { // 800x600yuv422
        if (mCamera->SetVideoFormat(1) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(0) != CAM_SUCCESS) return false;
        mChannels = 2;
    }
    else if (mCamera->HasVideoMode(0, 3)) { // 640x480yuv422
        if (mCamera->SetVideoFormat(0) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(3) != CAM_SUCCESS) return false;
        mChannels = 2;
    }
    else if (mCamera->HasVideoMode(0, 1)) { // 320x240yuv422
        if (mCamera->SetVideoFormat(0) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(1) != CAM_SUCCESS) return false;
        mChannels = 2;
    */
    }
    else if (mCamera->HasVideoMode(2, 5)) { // 1600x1200mono
        if (mCamera->SetVideoFormat(2) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(5) != CAM_SUCCESS) return false;
        mChannels = 1;
    }
    else if (mCamera->HasVideoMode(2, 2)) { // 1280x960mono
        if (mCamera->SetVideoFormat(2) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(2) != CAM_SUCCESS) return false;
        mChannels = 1;
    }
    else if (mCamera->HasVideoMode(1, 5)) { // 1024x768mono
        if (mCamera->SetVideoFormat(1) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(5) != CAM_SUCCESS) return false;
        mChannels = 1;
    }
    else if (mCamera->HasVideoMode(1, 2)) { // 800x600mono
        if (mCamera->SetVideoFormat(1) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(2) != CAM_SUCCESS) return false;
        mChannels = 1;
    }
    else if (mCamera->HasVideoMode(0, 5)) { // 640x480mono
        if (mCamera->SetVideoFormat(0) != CAM_SUCCESS) return false;
        if (mCamera->SetVideoMode(5) != CAM_SUCCESS) return false;
        mChannels = 1;
    }
    else {
        return false;
    }

    mCamera->GetVideoFrameDimensions(&mXResolution, &mYResolution);
    mReturnFrame = cvCreateImage(cvSize(mXResolution, mYResolution), IPL_DEPTH_8U, mChannels);
    mCamera->StartImageAcquisitionEx(6, 1, ACQ_START_VIDEO_STREAM);
    mIsCapturing = true;

    return isCapturing();
}

void CaptureCmu::stop()
{
    if (isCapturing()) {
        mCamera->StopImageAcquisition();
        cvReleaseImage(&mReturnFrame);
        mIsCapturing = false;
    }
}

IplImage *CaptureCmu::captureImage()
{
    if (!isCapturing()) {
        return NULL;
    }

    if (mCamera->AcquireImageEx(false, NULL) == CAM_SUCCESS) {
        unsigned long length = mReturnFrame->widthStep * mYResolution;
        memcpy(mReturnFrame->imageData, mCamera->GetRawData(&length), length);
    }
    return mReturnFrame;
}

bool CaptureCmu::showSettingsDialog()
{
    CameraControlDialog(NULL, mCamera, true);
    return true;
}

std::string CaptureCmu::SerializeId()
{
    return "CaptureCmu";
}

bool CaptureCmu::Serialize(Serialization *serialization)
{
    if (!mCamera) {
        return false;
    }

	unsigned short value;
	if (serialization->IsInput()) {
		if (!serialization->Serialize(value, "Gain")) return false;
		mCamera->GetCameraControl(FEATURE_GAIN)->SetAutoMode(false);
		mCamera->GetCameraControl(FEATURE_GAIN)->SetValue(value);

		if (!serialization->Serialize(value, "AutoExposure")) return false;
		mCamera->GetCameraControl(FEATURE_AUTO_EXPOSURE)->SetAutoMode(false);
		mCamera->GetCameraControl(FEATURE_AUTO_EXPOSURE)->SetValue(value);

		if (!serialization->Serialize(value, "Shutter")) return false;
		mCamera->GetCameraControl(FEATURE_SHUTTER)->SetAutoMode(false);
		mCamera->GetCameraControl(FEATURE_SHUTTER)->SetValue(value);

		if (!serialization->Serialize(value, "Brightness")) return false;
		mCamera->GetCameraControl(FEATURE_BRIGHTNESS)->SetAutoMode(false);
		mCamera->GetCameraControl(FEATURE_BRIGHTNESS)->SetValue(value);

		if (!serialization->Serialize(value, "Gamma")) return false;
		mCamera->GetCameraControl(FEATURE_GAMMA)->SetAutoMode(false);
		mCamera->GetCameraControl(FEATURE_GAMMA)->SetValue(value);
	} else {
		mCamera->GetCameraControl(FEATURE_GAIN)->GetValue(&value);
		if (!serialization->Serialize(value, "Gain")) return false;

		mCamera->GetCameraControl(FEATURE_AUTO_EXPOSURE)->GetValue(&value);
		if (!serialization->Serialize(value, "AutoExposure")) return false;

		mCamera->GetCameraControl(FEATURE_SHUTTER)->GetValue(&value);
		if (!serialization->Serialize(value, "Shutter")) return false;

		mCamera->GetCameraControl(FEATURE_BRIGHTNESS)->GetValue(&value);
		if (!serialization->Serialize(value, "Brightness")) return false;

		mCamera->GetCameraControl(FEATURE_GAMMA)->GetValue(&value);
		if (!serialization->Serialize(value, "Gamma")) return false;
	}
	return true;
}

CapturePluginCmu::CapturePluginCmu(const std::string &captureType)
    : CapturePlugin(captureType)
{
}

CapturePluginCmu::~CapturePluginCmu()
{
}

CapturePlugin::CaptureDeviceVector CapturePluginCmu::enumerateDevices()
{
    CaptureDeviceVector devices;

    C1394Camera camera;
    if (camera.CheckLink() != CAM_SUCCESS) {
        return devices;
    }

    int numberCameras = camera.GetNumberCameras();
    for (int i = 0; i < numberCameras; i++) {
        camera.SelectCamera(i);
        LARGE_INTEGER uniqueId;
        camera.GetCameraUniqueID(&uniqueId);
        std::stringstream convert;
        convert << std::hex << uniqueId.HighPart << uniqueId.LowPart;
        std::stringstream description;
        char buffer[500];
        camera.GetCameraVendor(buffer, 500);
        description << buffer << " ";
        camera.GetCameraName(buffer, 500);
        description << buffer;
        CaptureDevice captureDevice(mCaptureType, convert.str(), description.str());
        devices.push_back(captureDevice);
    }

    return devices;
}

Capture *CapturePluginCmu::createCapture(const CaptureDevice captureDevice)
{
    return new CaptureCmu(captureDevice);
}

void registerPlugin(const std::string &captureType, alvar::CapturePlugin *&capturePlugin)
{
    capturePlugin = new CapturePluginCmu(captureType);
}

} // namespace plugins
} // namespace alvar

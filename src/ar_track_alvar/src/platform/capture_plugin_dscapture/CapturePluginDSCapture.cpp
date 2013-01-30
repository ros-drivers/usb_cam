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

#include "CapturePluginDSCapture.h"

#include <sstream>

using namespace std;

namespace alvar {
namespace plugins {

CaptureDSCapture::CaptureDSCapture(const CaptureDevice captureDevice)
    : Capture(captureDevice)
    , sampler(this)
    , m_pDSCapture(NULL)
    , m_nBpp(0)
    , m_nVideo_x_res(-1)
    , m_nVideo_y_res(-1)
    , imgBuffer(NULL)
    , imgBufferForCallback(NULL)
    , mReturnFrame(NULL)
{
  InitializeCriticalSection(&crit);
  next_event = CreateEvent(NULL, FALSE, FALSE, NULL);
  m_pDSCapture = new CDSCapture(true, false);
}

CaptureDSCapture::~CaptureDSCapture()
{
  stop();
  if (m_pDSCapture) {
    m_pDSCapture->Stop();
    delete m_pDSCapture; 
  }
  delete imgBuffer;
  delete imgBufferForCallback;
  DeleteCriticalSection(&crit);
}

bool CaptureDSCapture::start()
{
    if (m_pDSCapture) {
	HRESULT hr = m_pDSCapture->Init(true, false, mCaptureDevice.id().c_str(), NULL);

	if(SUCCEEDED(hr)){
		// Set video grabber media type
		AM_MEDIA_TYPE mt;
		ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
		mt.majortype = MEDIATYPE_Video;
		mt.subtype = MEDIASUBTYPE_RGB24;
		hr = m_pDSCapture->SetVideoGrabberFormat(&mt);
	}

	if(SUCCEEDED(hr))
		hr = m_pDSCapture->ShowVideoCaptureFormatDialog();

	// We must connect filters before we can get connected media type from grabber(s)
	if(SUCCEEDED(hr))
		hr = m_pDSCapture->Connect();

	if(SUCCEEDED(hr)){
		AM_MEDIA_TYPE mt;
		ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
		HRESULT hr = m_pDSCapture->GetVideoGrabberFormat(&mt);
		if(SUCCEEDED(hr)){
			// Examine the format block.
			if ((mt.formattype == FORMAT_VideoInfo) && 
				(mt.cbFormat >= sizeof(VIDEOINFOHEADER)) &&
				(mt.pbFormat != NULL) ) 
			{
				if(mt.subtype == MEDIASUBTYPE_RGB24)
					m_nBpp = 24;
				else if(mt.subtype == MEDIASUBTYPE_RGB32)
					m_nBpp = 32;

				VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)mt.pbFormat;
				//cout << "Video capture: "<<pVih->bmiHeader.biWidth<<"x"<<pVih->bmiHeader.biHeight<<" "<<pVih->bmiHeader.biBitCount<<" bpp "<<fps<<" fps";
				m_nVideo_x_res = pVih->bmiHeader.biWidth;
				m_nVideo_y_res = pVih->bmiHeader.biHeight;
			}
		}
	}

	if(FAILED(hr)){
		return false;
	}

	m_pDSCapture->AddVideoCallback(&sampler);

        buffer_size = (int)(m_nVideo_x_res * m_nVideo_y_res * (m_nBpp/8.0));
        imgBuffer = new BYTE[buffer_size];
        imgBufferForCallback = new BYTE[buffer_size];
        mReturnFrame = cvCreateImageHeader(cvSize(m_nVideo_x_res, m_nVideo_y_res), IPL_DEPTH_8U,m_nBpp / 8);
        mReturnFrame->imageData = (char*)imgBuffer;
    }
    m_pDSCapture->Start();
    mIsCapturing = true;
    return true;
}

void CaptureDSCapture::stop()
{
    if (isCapturing()) {
      HRESULT hr = m_pDSCapture->Stop();
      mIsCapturing = false;
    }
}

IplImage *CaptureDSCapture::captureImage()
{
    if (!isCapturing()) {
        return NULL;
    }

    IplImage *ret = NULL;
    if (WaitForSingleObject(next_event, 1000) == WAIT_OBJECT_0) {
      EnterCriticalSection(&crit);
      ret = mReturnFrame;
      ret->origin = 1;
      memcpy(imgBuffer,imgBufferForCallback,buffer_size);
      LeaveCriticalSection(&crit);
    }
  return ret;
}

bool CaptureDSCapture::showSettingsDialog()
{
    HRESULT hr = m_pDSCapture->ShowVideoCaptureFormatDialog();
    return SUCCEEDED(hr);
}

string CaptureDSCapture::SerializeId()
{
    return "CaptureDSCapture";
}

bool CaptureDSCapture::Serialize(Serialization *serialization)
{
    return false;
}

void CaptureDSCapture::OnVideoSample(BYTE* pBuffer, DWORD dwDataLen, REFERENCE_TIME t_start)
{
        EnterCriticalSection(&crit);
	if(pBuffer){
          if (dwDataLen <= buffer_size) {
            memcpy(imgBufferForCallback,pBuffer,dwDataLen);
          }
          SetEvent(next_event);
	}
        LeaveCriticalSection(&crit);
}


CapturePluginDSCapture::CapturePluginDSCapture(const string &captureType)
    : CapturePlugin(captureType)
{
}

CapturePluginDSCapture::~CapturePluginDSCapture()
{
}

CapturePlugin::CaptureDeviceVector CapturePluginDSCapture::enumerateDevices()
{
    CaptureDeviceVector devices;
    CDSCapture capture;
    device_map* vids = capture.GetVideoCaptureDevices();
    for (device_map::iterator i = vids->begin(); i != vids->end(); ++i) {
      CaptureDevice dev("dscapture", i->first, i->second);
      devices.push_back(dev);
    }
    
    return devices;
}

Capture *CapturePluginDSCapture::createCapture(const CaptureDevice captureDevice)
{
    return new CaptureDSCapture(captureDevice);
}

void registerPlugin(const string &captureType, alvar::CapturePlugin *&capturePlugin)
{
    capturePlugin = new CapturePluginDSCapture(captureType);
}

} // namespace plugins
} // namespace alvar

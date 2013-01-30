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

#ifndef CAPTUREPLUGINDSCAPTURE_H
#define CAPTUREPLUGINDSCAPTURE_H

/**
 * \file CapturePluginDSCapture.h
 *
 * \brief This file implements a capture plugin based on DSCapture.
 *
 * \note The DSCapture plugin is currently experimental and not included in
 *       the build by default.
 */

#ifdef WIN32
    #ifdef ALVAR_Capture_Plugin_DSCapture_BUILD
        #define ALVAR_CAPTURE_PLUGIN_DSCAPTURE_EXPORT __declspec(dllexport)
    #else
        #define ALVAR_CAPTURE_PLUGIN_DSCAPTURE_EXPORT __declspec(dllimport)
    #endif
#else
    #define ALVAR_CAPTURE_PLUGIN_DSCAPTURE_EXPORT
#endif

#include "Capture.h"
#include "CapturePlugin.h"

#include "dscapture.h"

namespace alvar {
  
/**
 * \brief Dynamically loaded plugins namespace.
 */
namespace plugins {

/**
 * \brief Implementation of Capture interface for DSCapture plugin.
 *
 * \note The DSCapture plugin is currently experimental and not included in
 *       the build by default.
 */
class ALVAR_CAPTURE_PLUGIN_DSCAPTURE_EXPORT CaptureDSCapture
    : public alvar::Capture
{
    class VideoSampler : public IVideoCallback {
    public:
      CaptureDSCapture *parent;
      VideoSampler(CaptureDSCapture *_parent) : parent(_parent) {}
      void OnVideoSample(BYTE* pBuffer, DWORD dwDataLen, REFERENCE_TIME t_start) {
        parent->OnVideoSample(pBuffer, dwDataLen, t_start);
      }
      bool operator=(const VideoSampler &vs) { return parent == vs.parent; }
    } sampler;
    friend class VideoSampler;
    
    void OnVideoSample(BYTE* pBuffer, DWORD dwDataLen, REFERENCE_TIME t_start);
    
public:
    
    /**
     * \brief Constructor.
     *
     * \param captureDevice Information of which camera to create.
     */
    CaptureDSCapture(const CaptureDevice captureDevice);
    /**
     * \brief Destructor.
     */
    ~CaptureDSCapture();
    bool start();
    void stop();
    IplImage *captureImage();
    bool showSettingsDialog();
    std::string SerializeId();
    bool Serialize(Serialization *serialization);
    

private:
    CDSCapture  *m_pDSCapture;
    int         m_nBpp;
    int         m_nVideo_x_res;
    int         m_nVideo_y_res;
    BYTE        *imgBuffer;
    BYTE        *imgBufferForCallback;
    IplImage    *mReturnFrame;
    CRITICAL_SECTION crit;
    unsigned int buffer_size;
    HANDLE next_event;
};

/**
 * \brief Implementation of CapturePlugin interface for DSCapture plugin.
 *
 * \note The DSCapture plugin is currently experimental and not included in
 *       the build by default.
 */
class ALVAR_CAPTURE_PLUGIN_DSCAPTURE_EXPORT CapturePluginDSCapture
    : public alvar::CapturePlugin
{
public:
    /**
     * \brief Constructor.
     *
     * \param captureType A unique identifier for the capture plugin.
     */
    CapturePluginDSCapture(const std::string &captureType);
    /**
     * \brief Destructor.
     */
    ~CapturePluginDSCapture();
    CaptureDeviceVector enumerateDevices();
    Capture *createCapture(const CaptureDevice captureDevice);
};

extern "C" ALVAR_CAPTURE_PLUGIN_DSCAPTURE_EXPORT void registerPlugin(const std::string &captureType, alvar::CapturePlugin *&capturePlugin);

} // namespace plugins
} // namespace alvar

#endif

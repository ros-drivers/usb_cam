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

#ifndef CAPTUREPLUGINCMU_H
#define CAPTUREPLUGINCMU_H

/**
 * \file CapturePluginCmu.h
 *
 * \brief This file implements a capture plugin based on Cmu.
 */

#ifdef WIN32
    #ifdef ALVAR_Capture_Plugin_CMU_BUILD
        #define ALVAR_CAPTURE_PLUGIN_CMU_EXPORT __declspec(dllexport)
    #else
        #define ALVAR_CAPTURE_PLUGIN_CMU_EXPORT __declspec(dllimport)
    #endif
#else
    #define ALVAR_CAPTURE_PLUGIN_CMU_EXPORT
#endif

#include "Capture.h"
#include "CapturePlugin.h"

#include "1394camera.h"

namespace alvar {
  
/**
 * \brief Dynamically loaded plugins namespace.
 */
namespace plugins {

/**
 * \brief Implementation of Capture interface for Cmu plugin.
 */
class ALVAR_CAPTURE_PLUGIN_CMU_EXPORT CaptureCmu
    : public alvar::Capture
{
public:
    /**
     * \brief Constructor.
     *
     * \param captureDevice Information of which camera to create.
     */
    CaptureCmu(const CaptureDevice captureDevice);
    /**
     * \brief Destructor.
     */
    ~CaptureCmu();
    bool start();
    void stop();
    IplImage *captureImage();
    bool showSettingsDialog();
	std::string SerializeId();
	bool Serialize(Serialization *serialization);
private:
    C1394Camera *mCamera;
    int mChannels;
    IplImage *mReturnFrame;
};

/**
 * \brief Implementation of CapturePlugin interface for Cmu plugin.
 */
class ALVAR_CAPTURE_PLUGIN_CMU_EXPORT CapturePluginCmu
    : public alvar::CapturePlugin
{
public:
    /**
     * \brief Constructor.
     *
     * \param captureType A unique identifier for the capture plugin.
     */
    CapturePluginCmu(const std::string &captureType);
    /**
     * \brief Destructor.
     */
    ~CapturePluginCmu();
    CaptureDeviceVector enumerateDevices();
    Capture *createCapture(const CaptureDevice captureDevice);
};

extern "C" ALVAR_CAPTURE_PLUGIN_CMU_EXPORT void registerPlugin(const std::string &captureType, alvar::CapturePlugin *&capturePlugin);

} // namespace plugins
} // namespace alvar

#endif

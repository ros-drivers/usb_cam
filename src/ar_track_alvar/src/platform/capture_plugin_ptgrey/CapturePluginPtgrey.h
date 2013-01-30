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

#ifndef CAPTUREPLUGINPTGREY_H
#define CAPTUREPLUGINPTGREY_H

/**
 * \file CapturePluginPtgrey.h
 *
 * \brief This file implements a capture plugin based on Ptgrey.
 *
 * \note The PointGrey plugin is currently experimental and not included in
 *       the build by default.
 */

#ifdef WIN32
    #ifdef ALVAR_Capture_Plugin_Ptgrey_BUILD
        #define ALVAR_CAPTURE_PLUGIN_PTGREY_EXPORT __declspec(dllexport)
    #else
        #define ALVAR_CAPTURE_PLUGIN_PTGREY_EXPORT __declspec(dllimport)
    #endif
#else
    #define ALVAR_CAPTURE_PLUGIN_PTGREY_EXPORT
#endif

#include "Capture.h"
#include "CapturePlugin.h"

#include "FlyCapture2Defs.h"

// Forward declaration for PTGrey specific classes.
namespace FlyCapture2 {
  class Camera; 
  class Image;
}

namespace alvar {
  
/**
 * \brief Dynamically loaded plugins namespace.
 */
namespace plugins {

/**
 * \brief Implementation of Capture interface for Ptgrey plugin.
 *
 * \note The PointGrey plugin is currently experimental and not included in
 *       the build by default.
 */
class ALVAR_CAPTURE_PLUGIN_PTGREY_EXPORT CapturePtgrey
    : public alvar::Capture
{
public:
    /**
     * \brief Constructor.
     *
     * \param captureDevice Information of which camera to create.
     */
    CapturePtgrey(const CaptureDevice captureDevice);
    /**
     * \brief Destructor.
     */
    ~CapturePtgrey();
    bool start();
    void stop();
    IplImage *captureImage();
    bool showSettingsDialog();
    std::string SerializeId();
    bool Serialize(Serialization *serialization);
private:
    FlyCapture2::Camera *mCamera;
    FlyCapture2::Image *mImage;
    FlyCapture2::PGRGuid mGUID;
    int mChannels;
    IplImage *mReturnFrame;
};

/**
 * \brief Implementation of CapturePlugin interface for Ptgrey plugin.
 *
 * \note The PointGrey plugin is currently experimental and not included in
 *       the build by default.
 */
class ALVAR_CAPTURE_PLUGIN_PTGREY_EXPORT CapturePluginPtgrey
    : public alvar::CapturePlugin
{
public:
    /**
     * \brief Constructor.
     *
     * \param captureType A unique identifier for the capture plugin.
     */
    CapturePluginPtgrey(const std::string &captureType);
    /**
     * \brief Destructor.
     */
    ~CapturePluginPtgrey();
    CaptureDeviceVector enumerateDevices();
    Capture *createCapture(const CaptureDevice captureDevice);
};

extern "C" ALVAR_CAPTURE_PLUGIN_PTGREY_EXPORT void registerPlugin(const std::string &captureType, alvar::CapturePlugin *&capturePlugin);

} // namespace plugins
} // namespace alvar

#endif

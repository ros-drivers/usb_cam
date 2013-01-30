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

#ifndef CAPTUREPLUGIN_H
#define CAPTUREPLUGIN_H

/**
 * \file CapturePlugin.h
 *
 * \brief This file implements a capture plugin interface.
 */

#include "Alvar.h"
#include "CaptureDevice.h"

namespace alvar {

/**
 * \brief CapturePlugin interface that plugins must implement.
 *
 * All plugins must implement the CapturePlugin interface. When the plugin is loaded,
 * the CapturePlugin implementation will register itself with the CaptureFactory.
 */
class ALVAR_EXPORT CapturePlugin
{
public:
    /**
     * \brief Constructor.
     *
     * \param captureType A unique identifier for the capture plugin.
     */
    CapturePlugin(const std::string &captureType)
        : mCaptureType(captureType)
    {
    }

    /**
     * \brief Destructor.
     */
    virtual ~CapturePlugin() {};

    /**
     * \brief Vector of CaptureDevices.
     */
    typedef std::vector<CaptureDevice> CaptureDeviceVector;

    /**
     * \brief Enumerate capture devices currently available.
     *
     * \return A vector of CaptureDevice objects that are currently available.
     */
    virtual CaptureDeviceVector enumerateDevices() = 0;

    /**
     * \brief Create Capture class. Transfers onwership to the caller.
     *
     * \param captureDevice Information of which camera to create.
     * \return A new Capture class for which the caller takes ownership.
     */
    virtual Capture *createCapture(const CaptureDevice captureDevice) = 0;

protected:
    std::string mCaptureType;
};

} // namespace alvar

#endif

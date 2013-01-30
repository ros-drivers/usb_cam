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

#ifndef CAPTUREDEVICE_H
#define CAPTUREDEVICE_H

/**
 * \file CaptureDevice.h
 *
 * \brief This file implements a capture device to hold camera information.
 */

#include "Alvar.h"

#include <string>

namespace alvar {

/**
 * \brief CaptureDevice holder for camera information.
 *
 * CaptureDevice contains the desired backend, the id and description of the camera.
 */
class ALVAR_EXPORT CaptureDevice
{
public:
    /**
     * \brief Constructor.
     *
     * \param captureType The type of capture backend.
     * \param id The id of the camera.
     * \param description A human readable description of the camera.
     */
    CaptureDevice(const std::string captureType, const std::string id, const std::string description = "");

    /**
     * \brief Destructor.
     */
    ~CaptureDevice();

    /**
     * \brief The type of capture backend.
     */
    std::string captureType() const;

    /**
     * \brief The id of the camera.
     */
    std::string id() const;

    /**
     * \brief The description of the camera.
     */
    std::string description() const;

    /**
     * \brief A unique name consisting of the capture type and the id.
     */
    std::string uniqueName() const;

private:
    std::string mCaptureType;
    std::string mId;
    std::string mDescription;
};

} // namespace alvar

#endif

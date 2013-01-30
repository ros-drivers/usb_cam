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

#include "CaptureDevice.h"

#include <sstream>

namespace alvar {

CaptureDevice::CaptureDevice(const std::string captureType, const std::string id, const std::string description)
    : mCaptureType(captureType)
    , mId(id)
    , mDescription(description)
{
}

CaptureDevice::~CaptureDevice()
{
}

std::string CaptureDevice::captureType() const
{
    return mCaptureType;
}

std::string CaptureDevice::id() const
{
    return mId;
}

std::string CaptureDevice::description() const
{
    return mDescription;
}

std::string CaptureDevice::uniqueName() const
{
    std::stringstream name;
    name << captureType() << "_" << id();
    return name.str();
}

} // namespace alvar

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

#ifndef CAPTURE_H
#define CAPTURE_H

/**
 * \file Capture.h
 *
 * \brief This file implements a capture interface.
 */

#include "Alvar.h"
#include "CaptureDevice.h"
#include "Util.h"

namespace alvar {

/**
 * \brief Capture interface that plugins must implement.
 *
 * All plugins must implement the Capture interface. This is the class that implements
 * all of the camera capture funtionality. This class is created by the CapturePlugin
 * implementation.
 */
class ALVAR_EXPORT Capture
{
public:
    /**
     * \brief Constructor.
     *
     * \param captureDevice Information of which camera to create.
     */
    Capture(const CaptureDevice captureDevice)
        : mCaptureDevice(captureDevice)
        , mXResolution(0)
        , mYResolution(0)
        , mIsCapturing(false)
    {
    }

    /**
     * \brief Destructor
     */
    virtual ~Capture() {}

    /**
     * \brief The camera information associated to this capture object.
     */
    CaptureDevice captureDevice() {return mCaptureDevice;}

    /**
     * \brief The resolution along the x axis (horizontal).
     */
    unsigned long xResolution() {return mXResolution;}

    /**
     * \brief The resolution along the y axis (vertical).
     */
    unsigned long yResolution() {return mYResolution;}

    /**
     * \brief Test if the camera was properly initialized.
     */
    bool isCapturing() {return mIsCapturing;}

    /**
     * \brief Set the resolution.
     *
     * \param xResolution The resolution along the x axis (horizontal).
     * \param yResolution The resolution along the y axis (vertical).
     */
    virtual void setResolution(const unsigned long xResolution, const unsigned long yResolution)
    {
    }

    /**
     * \brief Starts the camera capture.
     *
     * \return True if the camera was properly initialized, false otherwise.
     */
    virtual bool start() = 0;

    /**
     * \brief Stops the camera capture.
     */
    virtual void stop() = 0;

    /**
     * \brief Capture one image from the camera.
     *
     * Do not modify this image.
     *
     * \return The captured image.
     */
    virtual IplImage *captureImage() = 0;

    /**
     * \brief Save camera settings to a file.
     *
     * \param filename The filename to write to.
     * \return True if the settings were sucessfully saved, false otherwise.
     */
	virtual bool saveSettings(std::string filename) {
        if (!isCapturing()) {
            return false;
        }

		Serialization serialization(filename);
		try {
            serialization << (*this);
        }
		catch (...) {
            return false;
        }
		return true;
	}

    /**
     * \brief Load camera settings from a file.
     *
     * \param filename The filename to read from.
     * \return True if the settings were sucessfully loaded, false otherwise.
     */
    virtual bool loadSettings(std::string filename) {
        if (!isCapturing()) {
            return false;
        }

		Serialization serialization(filename);
		try {
            serialization >> (*this);
        }
		catch (...) {
            return false;
        }
		return true;
	}

    /**
     * \brief Show the settings dialog of the camera.
     * \return True if the settings dialog was shown, false otherwise.
     */
    virtual bool showSettingsDialog() = 0;

	/**
     * \brief The identification of the class for serialization.
     */
	virtual std::string SerializeId() = 0;

	/**
     * \brief Performs serialization of the class members and configuration.
     *
     * \param serialization The Serialization object.
     * \return True if the serialization of the class was successful, false otherwise.
     */
	virtual bool Serialize(Serialization *serialization) = 0;

protected:
    CaptureDevice mCaptureDevice;
    unsigned long mXResolution;
    unsigned long mYResolution;
    bool mIsCapturing;
};

} // namespace alvar

#endif

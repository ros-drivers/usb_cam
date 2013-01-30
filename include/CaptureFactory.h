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

#ifndef CAPTUREFACTORY_H
#define CAPTUREFACTORY_H

/**
 * \file CaptureFactory.h
 *
 * \brief This file implements a capture factory with a plugin interface to
 * allow for different capture backends to be loaded at runtime if the
 * platform supports them.
 */

#include "Alvar.h"
#include "Capture.h"
#include "CaptureDevice.h"
#include "Platform.h"
#include "Util.h"

namespace alvar {

class CaptureFactoryPrivate;

/**
 * \brief CaptureFactory for creating Capture classes.
 *
 * CaptureFactory is a singleton that creates Capture classes used to perform
 * camera acquisition. Different backends are implemented as dynamicly loaded plugins
 * so that platform dependancies can be handled at runtime and not compile time.
 */
class ALVAR_EXPORT CaptureFactory
{
public:
    /**
     * \brief The singleton instance of CaptureFactory.
     */
    static CaptureFactory *instance();

    /**
     * \brief Vector of strings.
     */
    typedef std::vector<std::string> CapturePluginVector;

    /**
     * \brief Enumerate capture plugins currently available.
     *
     * This method should be used carfully since it will load all the available plugins.
     *
     * \return A vector of strings identifying all currently available capture plugins.
     */
    CapturePluginVector enumeratePlugins();

    /**
     * \brief Vector of CaptureDevices.
     */
    typedef std::vector<CaptureDevice> CaptureDeviceVector;

    /**
     * \brief Enumerate capture devices currently available.
     *
     * This method should be used carfully since it will load all the known plugins
     * and call their respective enumeration methods. The vector of CaptureDevice
     * objects returned should be cached.
     *
     * \param captureType Force the enumeration of only one type of plugin.
     * \return A vector of CaptureDevice objects that are currently available.
     */
    CaptureDeviceVector enumerateDevices(const std::string &captureType = "");

    /**
     * \brief Create Capture class. Transfers onwership to the caller.
     *
     * If the needed backend plugin is not loaded, an attempt is made to load it and
     * an instance of it is kept such that it is available for subsequent calls.
     *
     * \param captureDevice CaptureDevice object specifying the plugin to use.
     * \return A new Capture class for which the caller takes ownership.
     */
    Capture *createCapture(const CaptureDevice captureDevice);

protected:
    /**
     * \brief Destructor.
     */
    ~CaptureFactory();

private:
    /**
     * \brief CaptureFactoryDestroyer for deleting the CaptureFactory singleton.
     */
    class CaptureFactoryDestroyer
    {
    public:
        CaptureFactoryDestroyer(CaptureFactory *instance = NULL) : mInstance(instance) {}
        ~CaptureFactoryDestroyer() {delete mInstance;}
        void set(CaptureFactory *instance) {mInstance = instance;}
    private:
        CaptureFactory *mInstance;
    };

    // private constructors and assignment operator for singleton implementation
    CaptureFactory();
    CaptureFactory(const CaptureFactory&);
    CaptureFactory &operator=(const CaptureFactory&);

    // static members for singleton implementation
    static CaptureFactory *mInstance;
    static Mutex mMutex;
    static CaptureFactoryDestroyer mDestroyer;

    // members
    CaptureFactoryPrivate *d;
};

} // namespace alvar

#endif

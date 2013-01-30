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

#include "CaptureFactory.h"

#include "CaptureFactory_private.h"
#include "CapturePlugin.h"
#include "DirectoryIterator.h"

namespace alvar {

CaptureFactoryPrivate::CaptureFactoryPrivate()
    : mPluginPaths()
    , mPluginPrefix()
    , mPluginPostfix()
    , mLoadedAllPlugins(false)
    , mPluginMap()
    , mCapturePluginMap()
{
    setupPluginPaths();

    mPluginPrefix = pluginPrefix();
    mPluginPrefix.append("alvarcaptureplugin");

    mPluginPostfix.append(ALVAR_VERSION_NODOTS);
    #if _DEBUG
        mPluginPostfix.append("d");
    #endif
    mPluginPostfix.append(".");
    mPluginPostfix.append(pluginExtension());
}

CaptureFactoryPrivate::~CaptureFactoryPrivate()
{
    for (CapturePluginMap::iterator itr = mCapturePluginMap.begin(); itr != mCapturePluginMap.end(); itr++) {
        delete itr->second;
    }
    mCapturePluginMap.clear();
    mPluginMap.clear();
}

void CaptureFactoryPrivate::loadPlugins()
{
    // ensure that plugins have not already been loaded
    if (mLoadedAllPlugins) {
        return;
    }

    // iterate over search paths
    for (PluginPathsVector::iterator itr = mPluginPaths.begin(); itr != mPluginPaths.end(); ++itr) {
        DirectoryIterator directory(*itr);

        // iterate over entries in current path
        while (directory.hasNext()) {
            std::string entry = directory.next();

            // verify that filename matches the plugin convention
            int prefixIndex = entry.find(mPluginPrefix);
            int postfixIndex = entry.rfind(mPluginPostfix);
            if (prefixIndex == -1 || postfixIndex == -1) {
                continue;
            }

            // load the actual plugin
            entry = entry.substr(mPluginPrefix.size(), postfixIndex - mPluginPrefix.size());
            loadPlugin(entry, directory.currentPath());
        }
    }

    // this should only be done once
    mLoadedAllPlugins = true;
}

void CaptureFactoryPrivate::loadPlugin(const std::string &captureType)
{
    // ensure plugin is not alredy loaded
    if (mPluginMap.find(captureType) != mPluginMap.end()) {
        return;
    }

    // iterate over search paths
    for (PluginPathsVector::iterator itr = mPluginPaths.begin(); itr != mPluginPaths.end(); ++itr) {
        DirectoryIterator directory(*itr);
        
        // iterate over entries in current path
        while (directory.hasNext()) {
            std::string entry = directory.next();

            // verify that filename matches the plugin convention
            int prefixIndex = entry.find(mPluginPrefix);
            int postfixIndex = entry.rfind(mPluginPostfix);
            if (prefixIndex == -1 || postfixIndex == -1) {
                continue;
            }

            // verify that filename matches capture type
            entry = entry.substr(mPluginPrefix.size(), postfixIndex - mPluginPrefix.size());
            if (entry != captureType) {
                continue;
            }
            
            // load the actual plugin
            loadPlugin(entry, directory.currentPath());

            // stop searching
            break;
        }
    }
}

void CaptureFactoryPrivate::loadPlugin(const std::string &captureType, const std::string &filename)
{
    // ensure plugin is not alredy loaded
    if (mPluginMap.find(captureType) != mPluginMap.end()) {
        return;
    }

    try {
        // create and load the plugin
        Plugin plugin(filename);

        // register the plugin
        // for this to work, each plugin must export the following method
        //   extern "C" __declspec(dllexport) void registerPlugin(const std::string &captureType, alvar::CapturePlugin *capturePlugin);
        // which creates a new CapturePlugin object: capturePlugin = new MyCapturePlugin(captureType);
        typedef void (*RegisterPlugin)(const std::string &captureType, CapturePlugin *&capturePlugin);
        RegisterPlugin registerPlugin = (RegisterPlugin)plugin.resolve("registerPlugin");
        CapturePlugin *capturePlugin = NULL;
        if (registerPlugin) {
            registerPlugin(captureType, capturePlugin);
        }

        // return if plugin did not create it's capture plugin
        if (capturePlugin == NULL) {
            return;
        }

        // insert the plugin and capture plugin into maps
        mPluginMap.insert(PluginMap::value_type(captureType, plugin));
        mCapturePluginMap.insert(CapturePluginMap::value_type(captureType, capturePlugin));
    }
    catch (AlvarException e) {
        // if anything fails, simply ignore it...
        #if defined(_DEBUG) || !defined(NDEBUG)
            std::cout << e.what() << std::endl;
        #endif
    }
}

CapturePlugin *CaptureFactoryPrivate::getPlugin(const std::string &captureType)
{
    // find CapturePlugin implementation according to capture type
    CapturePluginMap::iterator itr;
    itr = mCapturePluginMap.find(captureType);

    // if not found, attempt to load plugin
    if (itr == mCapturePluginMap.end()) {
        loadPlugin(captureType);
        itr = mCapturePluginMap.find(captureType);
    }

    // return CapturePlugin implementation if found
    CapturePlugin *capturePlugin = NULL;
    if (itr != mCapturePluginMap.end()) {
        capturePlugin = itr->second;
    }
    return capturePlugin;
}


// static class variables instantiation for singleton implementation
CaptureFactory *CaptureFactory::mInstance = NULL;
Mutex CaptureFactory::mMutex;
CaptureFactory::CaptureFactoryDestroyer CaptureFactory::mDestroyer;

CaptureFactory *CaptureFactory::instance()
{
    // do not use double-checked locking
    // http://www.aristeia.com/Papers/DDJ_Jul_Aug_2004_revised.pdf
    // use a destroyer class to properly clean up resources
    // http://www.research.ibm.com/designpatterns/pubs/ph-jun96.txt
    Lock lock(&mMutex);
    if (mInstance == NULL) {
        mInstance = new CaptureFactory();
        mDestroyer.set(mInstance);
    }
    return mInstance;
}

CaptureFactory::CaptureFactory()
    : d(new CaptureFactoryPrivate())
{
}

CaptureFactory::~CaptureFactory()
{
    delete d;
}

CaptureFactory::CapturePluginVector CaptureFactory::enumeratePlugins()
{
    // load all plugins
    d->loadPlugins();

    // return the available plugins as a vector of plugin names
    CaptureFactory::CapturePluginVector keys;
    for (CaptureFactoryPrivate::PluginMap::iterator itr = d->mPluginMap.begin(); itr != d->mPluginMap.end(); ++itr) {
        keys.push_back(itr->first);
    }

    return keys;
}

CaptureFactory::CaptureDeviceVector CaptureFactory::enumerateDevices(const std::string &captureType)
{
    CaptureDeviceVector devices;

    // load all plugins and enumerate their devices
    if (captureType.empty()) {
        d->loadPlugins();
        for (CaptureFactoryPrivate::CapturePluginMap::iterator itr = d->mCapturePluginMap.begin(); itr != d->mCapturePluginMap.end(); ++itr) {
            CaptureDeviceVector pluginDevices = itr->second->enumerateDevices();
            devices.insert(devices.end(), pluginDevices.begin(), pluginDevices.end());
        }
    }
    // only enumerate the devices of one plugin
    else {
        CapturePlugin *capturePlugin = d->getPlugin(captureType);
        if (capturePlugin) {
            devices = capturePlugin->enumerateDevices();
        }
    }

    return devices;
}

Capture *CaptureFactory::createCapture(const CaptureDevice captureDevice)
{
    // get CapturePlugin implementation
    CapturePlugin *capturePlugin = d->getPlugin(captureDevice.captureType());

    // create Capture implementation and return
    Capture *capture = NULL;
    if (capturePlugin) {
        capture = capturePlugin->createCapture(captureDevice);
    }
    return capture;
}

} // namespace alvar

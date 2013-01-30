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

#ifndef CAPTUREFACTORY_PRIVATE_H
#define CAPTUREFACTORY_PRIVATE_H

#include <vector>
#include <map>
#include <string>

#include "Plugin.h"

namespace alvar {

class CapturePlugin;

class CaptureFactoryPrivate
{
public:
    CaptureFactoryPrivate();
    ~CaptureFactoryPrivate();

    void setupPluginPaths();
    void parseEnvironmentVariable(const std::string &variable);
    std::string pluginPrefix();
    std::string pluginExtension();

    void loadPlugins();
    void loadPlugin(const std::string &captureType);
    void loadPlugin(const std::string &captureType, const std::string &filename);
    CapturePlugin *getPlugin(const std::string &captureType);

    typedef std::vector<std::string> PluginPathsVector;
    PluginPathsVector mPluginPaths;
    std::string mPluginPrefix;
    std::string mPluginPostfix;

    bool mLoadedAllPlugins;
    typedef std::map<std::string, Plugin> PluginMap;
    PluginMap mPluginMap;
    typedef std::map<std::string, CapturePlugin *> CapturePluginMap;
    CapturePluginMap mCapturePluginMap;
};

} // namespace alvar

#endif

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

#include "CaptureFactory_private.h"

#include <windows.h>

namespace alvar {

void CaptureFactoryPrivate::setupPluginPaths()
{
    // application path and default plugin path
    const DWORD bufferSize = 4096;
    char applicationBuffer[bufferSize];
    DWORD count = GetModuleFileName(NULL, applicationBuffer, bufferSize);
    if (count != 0 && count < bufferSize) {
        std::string applicationPath(applicationBuffer, count);
        applicationPath = std::string(applicationPath, 0, applicationPath.find_last_of("\\"));
        mPluginPaths.push_back(applicationPath);
        mPluginPaths.push_back(applicationPath + "\\alvarplugins");
    }

    // ALVAR library path
    parseEnvironmentVariable(std::string("ALVAR_LIBRARY_PATH"));

    // ALVAR plugin path
    parseEnvironmentVariable(std::string("ALVAR_PLUGIN_PATH"));
}

void CaptureFactoryPrivate::parseEnvironmentVariable(const std::string &variable)
{
    // acquire environment variable
    char *buffer;
    std::string path("");
    #if defined(_MSC_VER) && (_MSC_VER < 1400)
        buffer = getenv(variable.data());
		if (buffer) {
			path = std::string(buffer);
		}
    #else
        size_t requiredSize;
        getenv_s(&requiredSize, NULL, 0, variable.data());
        if (requiredSize > 0) {
            buffer = (char *)malloc(requiredSize * sizeof(char));
            getenv_s(&requiredSize, buffer, requiredSize, variable.data());
            path = std::string(buffer, requiredSize - 1);
            free(buffer);
        }
    #endif

    // tokenize paths
    char delimitor = ';';
    if (!path.empty()) {
        std::string::size_type start = 0;
        std::string::size_type end = 0;
        while ((end = path.find_first_of(delimitor, start)) != std::string::npos) {
            std::string tmp(path, start, end - start);
            if (!tmp.empty()) {
                mPluginPaths.push_back(tmp);
            }
            start = end + 1;
        }
        if (start != path.size()) {
            std::string tmp(path, start, std::string::npos);
            if (!tmp.empty()) {
                mPluginPaths.push_back(tmp);
            }
        }
    }
}

std::string CaptureFactoryPrivate::pluginPrefix()
{
    return std::string("");
}

std::string CaptureFactoryPrivate::pluginExtension()
{
    return std::string("dll");
}

} // namespace alvar

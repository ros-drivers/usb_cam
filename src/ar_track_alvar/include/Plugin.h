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

#ifndef PLUGIN_H
#define PLUGIN_H

/**
 * \file Plugin.h
 *
 * \brief This file implements a loader for plugins as dynamic libraries.
 */

#include "Alvar.h"

#include <string>

namespace alvar {

class PluginPrivate;

/**
 * \brief Plugin for loading dynamic libraries.
 *
 * Plugin class for loading dynamic libraires. The library is loaded during construction and
 * unloaded during destruction.
 */
class Plugin
{
public:
    /**
     * \brief Constructor.
     *
     * Constructing a Plugin object will attempt to load the plugin dynamic library.
     *
     * \param filename The filename of the dynamic library to load.
     * \exception AlvarException An exeption is thrown if the library can't be loaded.
     */
    Plugin(const std::string filename);

    /**
     * \brief Copy constructor.
     *
     * \param plugin The Plugin to copy.
     */
    Plugin(const Plugin &plugin);

    /**
     * \brief Assignment operator.
     *
     * \param plugin The Plugin to copy.
     */
    Plugin &operator=(const Plugin &plugin);

    /**
     * \brief Destructor.
     */
    ~Plugin();

    /**
     * \brief Resolves the address of a symbol.
     *
     * The symbol must be exported from the library as a C function.
     *
     * \param symbol The signature of the symbol.
     * \return The address of the symbol.
     * \exception AlvarException An exception is thrown if the symbol is not found.
     */
    void *resolve(const char *symbol);

private:
    PluginPrivate *d;
    int *mReferenceCount;
};

} // namespace alvar

#endif

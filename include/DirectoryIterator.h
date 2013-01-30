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

#ifndef DIRECTORYITERATOR_H
#define DIRECTORYITERATOR_H

/**
 * \file DirectoryIterator.h
 *
 * \brief This file implements a directory iterator.
 */

#include "Alvar.h"

#include <string>

namespace alvar {

class DirectoryIteratorPrivate;

/**
 * \brief DirectoryIterator for iterating over files and directories.
 *
 * DirectoryIterator class for iterating over files and directories on the filesystem.
 */
class ALVAR_EXPORT DirectoryIterator
{
public:
    /**
     * \brief Constructor.
     *
     * \param path The path on the filesystem to iterate.
     */
    DirectoryIterator(const std::string &path);

    /**
     * \brief Destructor.
     */
    ~DirectoryIterator();

    /**
     * \brief Verifies if another entry exist in the directory.
     */
    bool hasNext();

    /**
     * \brief Advances the iterator and returns the name of the next entry.
     */
    std::string next();

    /**
     * \brief Returns the name of the current entry.
     */
    std::string currentEntry();

    /**
     * \brief Returns the path of the current entry.
     *
     * This appends the name of the current entry to the path of the directory that
     * is being iterated.
     */
    std::string currentPath();

private:
    DirectoryIteratorPrivate *d;
};

} // namespace alvar

#endif

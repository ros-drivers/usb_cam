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

#include "DirectoryIterator_private.h"

#include <windows.h>

namespace alvar {

class DirectoryIteratorPrivateData
{
public:
    DirectoryIteratorPrivateData()
        : mHandle(INVALID_HANDLE_VALUE)
        , mData()
    {
    }
    
    HANDLE mHandle;
    WIN32_FIND_DATA mData;
};

DirectoryIteratorPrivate::DirectoryIteratorPrivate(const std::string &path)
    : d(new DirectoryIteratorPrivateData())
    , mDirectory(path)
    , mEntry()
    , mValid(false)
{
    if (mDirectory.at(mDirectory.length() - 1) != '\\') {
        mDirectory.append("\\");
    }
}

DirectoryIteratorPrivate::~DirectoryIteratorPrivate()
{
    FindClose(d->mHandle);
    delete d;
}

bool DirectoryIteratorPrivate::hasNext()
{
    if (d->mHandle == INVALID_HANDLE_VALUE) {
        std::string search = mDirectory + "*";
        d->mHandle = FindFirstFile(search.data(), &d->mData);

        if (d->mHandle != INVALID_HANDLE_VALUE) {
            mValid = true;
            skip();
        }
    }

    return mValid;
}

std::string DirectoryIteratorPrivate::next()
{
    if (!hasNext()) {
        return "";
    }

    mEntry = std::string(d->mData.cFileName);

    if (!FindNextFile(d->mHandle, &d->mData)) {
        mValid = false;
    }
    else {
        skip();
    }

    return mEntry;
}

void DirectoryIteratorPrivate::skip()
{
    while (true) {
        if (std::string(d->mData.cFileName) != "." && std::string(d->mData.cFileName) != "..") {
            return;
        }

        if (!FindNextFile(d->mHandle, &d->mData)) {
            mValid = false;
            return;
        }
    }
}

} // namespace alvar

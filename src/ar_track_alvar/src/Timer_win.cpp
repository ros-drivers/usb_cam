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

#include "Timer_private.h"

#include <windows.h>

namespace alvar {

class TimerPrivateData
{
public:
    TimerPrivateData()
        : mPerformanceQuerySupported(false)
        , mPerformanceFrequency()
        , mPerformanceStart()
        , mStart()
    {
    }

    bool mPerformanceQuerySupported;
    LARGE_INTEGER mPerformanceFrequency;
    LARGE_INTEGER mPerformanceStart;
    DWORD mStart;
};

TimerPrivate::TimerPrivate()
    : d(new TimerPrivateData())
{
	QueryPerformanceFrequency(&d->mPerformanceFrequency);
	if (d->mPerformanceFrequency.QuadPart) {
        d->mPerformanceQuerySupported = true;
    }
}

TimerPrivate::~TimerPrivate()
{
    delete d;
}

void TimerPrivate::start()
{
    if (d->mPerformanceQuerySupported) {
        QueryPerformanceCounter(&d->mPerformanceStart);
    }
    else {
        d->mStart = GetTickCount();
    }
}

double TimerPrivate::stop()
{
    if (d->mPerformanceQuerySupported) {
        LARGE_INTEGER stop;
        LARGE_INTEGER difference;
        QueryPerformanceCounter(&stop);
        difference.QuadPart = stop.QuadPart - d->mPerformanceStart.QuadPart;
        return double(difference.QuadPart) / d->mPerformanceFrequency.QuadPart;
    }
    else {
        return (GetTickCount() - d->mStart) / 1000.0;
    }
}

} // namespace alvar

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

#include <time.h>

namespace alvar {

class TimerPrivateData
{
public:
    TimerPrivateData()
        : mStart()
    {
    }

    timespec mStart;
};

TimerPrivate::TimerPrivate()
    : d(new TimerPrivateData())
{
}

TimerPrivate::~TimerPrivate()
{
    delete d;
}

void TimerPrivate::start()
{
    clock_gettime(CLOCK_MONOTONIC, &d->mStart);
}

double TimerPrivate::stop()
{
    timespec stop;
    clock_gettime(CLOCK_MONOTONIC, &stop);
    return (stop.tv_sec - d->mStart.tv_sec) +
           (stop.tv_nsec - d->mStart.tv_nsec) / 1000000000.0;
}

} // namespace alvar

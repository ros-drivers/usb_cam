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

#include "Threads_private.h"

#include <vector>
#include <pthread.h>

namespace alvar {

class ThreadsPrivateData
{
public:
    ThreadsPrivateData()
        : mHandles()
    {
    }

    std::vector<pthread_t> mHandles;
};

ThreadsPrivate::ThreadsPrivate()
    : d(new ThreadsPrivateData())
{
}

ThreadsPrivate::~ThreadsPrivate()
{
    for (int i = 0; i < (int)d->mHandles.size(); ++i) {
		pthread_exit(&d->mHandles.at(i));
	}
	d->mHandles.clear();

    delete d;
}

bool ThreadsPrivate::create(void *(*method)(void *), void *parameters)
{
    pthread_t thread;
    if (pthread_create(&thread, 0, method, parameters)) {
        d->mHandles.push_back(thread);
        return true;
    }
    return false;
}

} // namespace alvar

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

#ifndef LOCK_H
#define LOCK_H

/**
 * \file Lock.h
 *
 * \brief This file implements a lock to simplify mutex handling.
 */

#include "Alvar.h"
#include "Mutex.h"
#include "Uncopyable.h"

namespace alvar {

/**
 * \brief Lock for simplifying mutex handling.
 *
 * Lock class for simplifying mutex handling. Simply pass a mutex to the
 * constructor and the mutex will be locked. When the lock is destroyed, the
 * mutex will be unlocked.
 */
class ALVAR_EXPORT Lock : private Uncopyable
{
public:
    /**
     * \brief Constructor.
     *
     * \param mutex The mutex to lock.
     */
    Lock(Mutex *mutex)
        : mMutex(mutex)
	{
        mMutex->lock();
	}

    /**
     * \brief Destructor.
     */
    ~Lock()
	{
        mMutex->unlock();
	}

private:
    Mutex *mMutex;
};

} // namespace alvar

#endif

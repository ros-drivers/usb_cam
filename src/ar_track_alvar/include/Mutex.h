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

#ifndef MUTEX_H
#define MUTEX_H

/**
 * \file Mutex.h
 *
 * \brief This file implements a mutex.
 */

#include "Alvar.h"

namespace alvar {

class MutexPrivate;

/**
 * \brief Mutex for synchronizing multiple threads.
 *
 * Mutex class for synchronizing multiple threads.
 */
class ALVAR_EXPORT Mutex
{
public:
    /**
     * \brief Constructor.
     */
    Mutex();

    /**
     * \brief Destructor.
     */
    ~Mutex();

    /**
     * \brief Locks the mutex.
     *
     * If the mutex is already locked by another thread, this method will
     * block until the other thread unlocks the mutex.
     */
    void lock();

    /**
     * \brief Unlocks the mutex.
     */
    void unlock();

private:
    MutexPrivate *d;
};

} // namespace alvar

#endif

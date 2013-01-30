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

#ifndef THREADS_H
#define THREADS_H

/**
 * \file Threads.h
 *
 * \brief This file implements a threads vector.
 */

#include "Alvar.h"

namespace alvar {

class ThreadsPrivate;

/**
 * \brief Threads vector for handling multiple threads.
 *
 * Threads class for handling multiple threads.
 */
class ALVAR_EXPORT Threads
{
public:
    /**
     * \brief Constructor.
     */
    Threads();

    /**
     * \brief Destructor.
     */
    ~Threads();

    /**
     * \brief Creates a new thread and returns true on success.
     *
     * \param method The method that the thread will execute.
     * \param parameters The parameters sent to the method.
     */
    bool create(void *(*method)(void *), void *parameters);

private:
    ThreadsPrivate *d;
};

} // namespace alvar

#endif

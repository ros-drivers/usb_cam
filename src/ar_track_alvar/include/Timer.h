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

#ifndef TIMER_H
#define TIMER_H

/**
 * \file Timer.h
 *
 * \brief This file implements a timer.
 */

#include "Alvar.h"

namespace alvar {

class TimerPrivate;

/**
 * \brief Timer for measuring execution time.
 *
 * Timer class for measuring execution time.
 */
class ALVAR_EXPORT Timer
{
public:
    /**
     * \brief Constructor.
     */
    Timer();

    /**
     * \brief Destructor.
     */
    ~Timer();

    /**
     * \brief Starts the timer.
     */
    void start();

    /**
     * \brief Stops the timer and returns the elapsed time in seconds.
     */
    double stop();

private:
    TimerPrivate *d;
};

} // namespace alvar

#endif

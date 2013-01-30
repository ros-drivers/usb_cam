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

#ifndef PLATFORM_H
#define PLATFORM_H

/**
 * \file Platform.h
 *
 * \brief This file implements generic platform specific methods.
 */

#include "Alvar.h"
#include "Lock.h"
#include "Mutex.h"
#include "Threads.h"
#include "Timer.h"
#include "Uncopyable.h"

namespace alvar {

/**
 * \brief Error reporting function inspired by error_at_line() on Linux.
 *
 * It flushes stdout and prints the filename, line number and printf
 * compatible error message to stderr. If error is specified, it also prints
 * the corresponding error message from strerror(). If status is not zero, it
 * exits the process.
 */
void ALVAR_EXPORT errorAtLine(int status, int error, const char *filename,
                              unsigned int line, const char *format, ...);

/**
 * \brief Sleep for a specified amount of milliseconds.
 */
void ALVAR_EXPORT sleep(unsigned long milliseconds);

} // namespace alvar

#ifdef min
	#undef min
#endif

#ifdef max
	#undef max
#endif

#endif

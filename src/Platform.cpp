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

#include "Platform.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#ifdef WIN32
    #include <windows.h>
#else
    #include <time.h>
#endif

namespace alvar {

void errorAtLine(int status, int error, const char *filename,
                 unsigned int line, const char *format, ...)
{
    fflush(stdout);
    if (filename) {
        fprintf(stderr, "%s:%d: ", filename, line);
    }
    if (format) {
        va_list args;
        va_start(args, format);
        vfprintf(stderr, format, args);
        va_end(args);
    }
    if (error) {
        fprintf(stderr, ": %s", strerror(error));
    }
    fprintf(stderr, "\n");
    fflush(stderr);
    if (status) {
        exit(status);
    }
}

void sleep(unsigned long milliseconds)
{
    #ifdef WIN32
        Sleep(milliseconds);
    #else
        struct timespec t;
        t.tv_sec = 0;
        t.tv_nsec = 1000 * 1000 * milliseconds;
        nanosleep(&t, NULL);
    #endif
}

} // namespace alvar

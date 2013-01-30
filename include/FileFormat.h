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

#ifndef FILEFORMAT_H
#define FILEFORMAT_H

/**
 * \file FileFormat.h
 *
 * \brief This file defines various file formats.
 */

namespace alvar {

	/**
	 * File format enumeration used when reading / writing configuration
	 * files.
	 */
	typedef enum {
		/**
		 * \brief Default file format.
		 *
		 * Format is either OPENCV, TEXT or XML depending on load/store function used.
		 */
		FILE_FORMAT_DEFAULT,

		/**
		 * \brief File format written with cvWrite.
		 *
		 * File contents depend on the specific load/store function used.
		 */
		FILE_FORMAT_OPENCV,

		/**
		 * \brief Plain ASCII text file format.
		 *
		 * File contents depend on the specific load/store function used.
		 */
		FILE_FORMAT_TEXT,

		/**
		 * \brief XML file format.
		 *
		 * XML schema depends on the specific load/store function used.
		 */
		FILE_FORMAT_XML

	} FILE_FORMAT;
}

#endif //FILEFORMAT_H

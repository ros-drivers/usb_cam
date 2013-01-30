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

#ifndef FILEFORMATUTILS_H
#define FILEFORMATUTILS_H

/**
 * \file FileFormatUtils.h
 *
 * \brief This file implements utilities that assist in reading and writing
 * files.
 */

#include "Alvar.h"
#include "cv.h"
#include "tinyxml.h"

namespace alvar {

	/** \brief Utility functions for file reading / writing.
	 */
	class ALVAR_EXPORT FileFormatUtils {
	private:
		/**
		 * \brief Reads matrix type, rows and cols from XML element.
		 * \return true if XML element appears to be valid; otherwise false.
		 */
		static bool decodeXMLMatrix(const TiXmlElement *xml_matrix, int &type, int &rows, int &cols);

	public:

		/** \brief Allocates CvMat of a correct type and size.
		 * \param xml_matrix alvar:matrix element.
		 * \return CvMat that has the correct size for \e parseXMLMatrix.
		 */
		static CvMat* allocateXMLMatrix(const TiXmlElement *xml_matrix);

		/** \brief Reads contents of alvar:matrix into CvMat.
		 *
		 * Parsing fails if the matrix is not the same type or does not have
		 * the same number of rows and columns as the XML element.
		 *
		 * \param xml_matrix alvar:matrix element. If NULL no parsing is done and
		 *                   false is returned.
		 * \param matrix CvMat that has the correct size, populated with data in
		 *               the xml_matrix.
		 * \return true if matrix was successfully parsed; otherwise false.
		 */
		static bool parseXMLMatrix(const TiXmlElement *xml_matrix, CvMat *matrix);

		/** \brief Allocates new XML element and populates it with a CvMat data.
		 *
		 * The returned element needs to be deallocated by the caller.
		 *
		 * \param element_name Name of the allocated tiXmlElement.
		 * \param matrix Data that is written into the returned XML element.
		 * \return Newly allocated TiXmlElement.
		 */
		static TiXmlElement* createXMLMatrix(const char* element_name, const CvMat *matrix);
	};
}

#endif //FILEFORMATUTILS_H

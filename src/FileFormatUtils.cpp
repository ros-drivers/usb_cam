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

#include "FileFormatUtils.h"
#include <stdio.h>
#include <sstream>
#include <limits>

namespace alvar {

	bool FileFormatUtils::decodeXMLMatrix(const TiXmlElement *xml_matrix, int &type, int &rows, int &cols) {
		const char * xml_type = xml_matrix->Attribute("type");
		if (strcmp("CV_32F", xml_type) == 0) type = CV_32F;
		else if (strcmp("CV_64F", xml_type) == 0) type = CV_64F;
		else return false;

		if (xml_matrix->QueryIntAttribute("rows", &rows) != TIXML_SUCCESS) return false;
		if (xml_matrix->QueryIntAttribute("cols", &cols) != TIXML_SUCCESS) return false;

		return true;
	}

	CvMat* FileFormatUtils::allocateXMLMatrix(const TiXmlElement *xml_matrix) {
		if (!xml_matrix) return NULL;

		int type, rows, cols;
		if (!decodeXMLMatrix(xml_matrix, type, rows, cols)) return NULL;

		return cvCreateMat(rows, cols, type);
	}

	bool FileFormatUtils::parseXMLMatrix(const TiXmlElement *xml_matrix, CvMat *matrix) {
		if (!xml_matrix || !matrix) return false;

		int type, rows, cols;
		if (!decodeXMLMatrix(xml_matrix, type, rows, cols)) return false;

		if (type != cvGetElemType(matrix)) return false;
		if (rows != matrix->rows) return false;
		if (cols != matrix->cols) return false;

		const TiXmlElement *xml_data = xml_matrix->FirstChildElement("data");
		for (int r = 0; r < matrix->rows; ++r) {
			for (int c = 0; c < matrix->cols; ++c) {
				if (!xml_data) return false;
				double value = atof(xml_data->GetText());
				cvSetReal2D(matrix, r, c, value);
				xml_data = (const TiXmlElement *) xml_data->NextSibling("data");
			}
		}

		return true;
	}

	TiXmlElement* FileFormatUtils::createXMLMatrix(const char* element_name, const CvMat *matrix) {
		if (!matrix) return NULL;

		TiXmlElement* xml_matrix = new TiXmlElement(element_name);
		int precision;
		if (cvGetElemType(matrix) == CV_32F) {
			xml_matrix->SetAttribute("type", "CV_32F");
			precision = std::numeric_limits<float>::digits10 + 2;
		}
		else if (cvGetElemType(matrix) == CV_64F) {
			xml_matrix->SetAttribute("type", "CV_64F");
			precision = std::numeric_limits<double>::digits10 + 2;
		}
		else {
			delete xml_matrix;
			return NULL;
		}

		xml_matrix->SetAttribute("rows", matrix->rows);
		xml_matrix->SetAttribute("cols", matrix->cols);

		for (int r = 0; r < matrix->rows; ++r) {
			for (int c = 0; c < matrix->cols; ++c) {
				TiXmlElement *xml_data = new TiXmlElement("data");
				xml_matrix->LinkEndChild(xml_data);
				std::stringstream ss;
				ss.precision(precision);
				ss<<cvGetReal2D(matrix, r, c);
				xml_data->LinkEndChild(new TiXmlText(ss.str().c_str()));
			}
		}
		return xml_matrix;
	}
}

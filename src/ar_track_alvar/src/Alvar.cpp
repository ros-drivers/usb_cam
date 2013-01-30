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

#include "Alvar.h"

#include <iostream>

namespace alvar {

void alvarInfo()
{
    std::cerr << "ALVAR " << ALVAR_VERSION << " - A Library for Virtual and Augmented Reality" << std::endl;
    std::cerr << "Copyright 2007-2012 VTT Technical Research Centre of Finland" << std::endl;
    std::cerr << "Licensed under the GNU Lesser General Public License" << std::endl;
    std::cerr << "Built on " << ALVAR_DATE << " for " << ALVAR_SYSTEM << std::endl;
    std::cerr << std::endl;
}

struct AlvarLoader {
	AlvarLoader() {
		alvarInfo();
	}
} alvarBasicLoader;

} // namespace alvar

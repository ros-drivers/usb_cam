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
#include "Filter.h"

template class ALVAR_EXPORT alvar::FilterArray<alvar::FilterAverage>;
template class ALVAR_EXPORT alvar::FilterArray<alvar::FilterMedian>;
template class ALVAR_EXPORT alvar::FilterArray<alvar::FilterRunningAverage>;
template class ALVAR_EXPORT alvar::FilterArray<alvar::FilterDoubleExponentialSmoothing>;

using namespace std;

namespace alvar {
using namespace std;

Filter::Filter() { 
	value=0; 
}

void FilterAverage::push_to_buffer(double y) {
	buffer.push_back(y);
	while (buffer.size() > window_size) {
		buffer.pop_front();
	}
}

double FilterAverage::next(double y) {
	if (window_size <= 0) {
		count++;
		double alpha = 1.0/count;
		return (value=((1.0-alpha)*value)+(alpha*y));
	} else {
		push_to_buffer(y);
		double sum = 0;
		for (deque<double>::iterator iter = buffer.begin(); iter != buffer.end(); iter++) {
			sum += (double)*iter;
		}
		return (value=sum/buffer.size());
	}
}

void FilterAverage::reset() {
	buffer.clear();
}

double FilterAverage::deviation() const {
	double sum = 0;
	if (buffer.size() == 0) return 0;
	for (deque<double>::const_iterator iter = buffer.begin(); iter != buffer.end(); iter++) {
		double val = ((double)*iter)-value;
		sum += (val*val);
	}
	sum /= buffer.size();
	return sqrt(sum);
}

double FilterMedian::next(double y) {
	if (window_size <= 1) return y;
	push_to_buffer(y);
	copy(buffer.begin(), buffer.end(), sort_buffer.begin());
	int nth = buffer.size()/2;
	nth_element(sort_buffer.begin(), sort_buffer.begin() + nth, sort_buffer.begin()+buffer.size());
	return value = sort_buffer[nth];
}

double FilterRunningAverage::next(double y) {
	if (breset) {
		breset=false;
		value=(double)y;
	} else {
		value = ((1.0-alpha) * value) + (alpha * (double)y);
	}
	return value;
}

void FilterRunningAverage::reset() { breset=true; }

double FilterDoubleExponentialSmoothing::next(double y) {
	if (breset) {
		breset=false;
		value=(double)y;
		slope=0.0;
	}
	else {
		double value_prev = value;
		value = ((1.0-alpha) * (value + slope)) + (alpha * (double)y);
		slope = ((1.0-gamma) * (slope)) + (gamma * (value - value_prev));
	}
	return value;
}

} // namespace alvar

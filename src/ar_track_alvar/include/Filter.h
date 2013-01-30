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

#ifndef FILTER_H
#define FILTER_H

/**
 * \file Filter.h
 *
 * \brief This file implements multiple filters.
 */

#include "Alvar.h"
#include <algorithm>
#include <deque>
#include <vector>
#include <cmath>

namespace alvar {

/**
 * \brief \e Filter is pure virtual class describing the basic virtual interface for all filters
 *
 * Basic usage:
 * \code
 *     FilterAverage a(3);
 *     cout<<a.next(3)<<endl;
 *     cout<<a.next(3.2)<<endl;
 *     cout<<a.next(1.3)<<endl;
 *     cout<<a.next(2.9)<<endl;
 * \endcode
 *
 * The Filter library provides also the assignment operators (operator=)
 * to simplify the process where you want to replace existing \e double's
 * in code with filtered values. For example if you want to filter
 * the d over time in the following code...
 * \code
 *     double d;
 *     cout<<(d = 3)<<endl;
 *     cout<<(d = 3.2)<<endl;
 *     cout<<(d = 1.3)<<endl;
 *     cout<<(d = 2.9)<<endl;
 * \endcode
 *
 * ...you can add filter just by replacing the \e double with the selected filter:
 * \code
 *     FilterDoubleExponentialSmoothing d(0.2,0.5);
 *     cout<<(d = 3)<<endl;
 *     cout<<(d = 3.2)<<endl;
 *     cout<<(d = 1.3)<<endl;
 *     cout<<(d = 2.9)<<endl;
 * \endcode
 *
 * However, using the assignment operator for setting time series data
 * is a little counter-intuitive, so please use \e next() instead when possible.
 *
 * \note   All inherited classes need to update \e value in \e next()
 */
class ALVAR_EXPORT Filter {
protected:
	double value;
public:
	/** \brief Constructor */
	Filter();
	/** \brief Get the latest value */
	double get() const { return value; }
	/** \brief Get the latest value */
	operator double () { return get(); }
	/** \brief Update the value. All inherited classes need to update \e value in \e next(). */
	virtual double next(double y) = 0;
	/** \brief Reset the filter state */
	virtual void reset() = 0;
};

/**
 * \brief \e FilterAverage provides an average filter
 *
 * The \e FilterAverage remembers \e window_size last elements
 * in the time series and returns always the average of these
 * elements. The size of the window \e window_size can be set
 * in the constructor or with \e setWindowSize() .
 * 
 * Note, that when the window_size is <= 0 we calculate the
 * average over the whole sequence without using the buffer.
 */
class ALVAR_EXPORT FilterAverage : public Filter {
protected:
	unsigned int count;
	unsigned int window_size;
	std::deque<double> buffer;
	void push_to_buffer(double y);
public:
	FilterAverage(int size=3) { setWindowSize(size); }
	void setWindowSize(int size) { window_size=size; count=0; }
	int getWindowSize() { return window_size; }
	int getCurrentSize() { return (int) buffer.size(); }
	double operator= (double _value) { return next(_value); }
	virtual double next(double y);
	virtual void reset();
	double deviation() const;
};

/**
 * \brief \e FilterMedian provides an median filter
 *
 * The \e FilterMedian remembers \e window_size last elements
 * in the time series and returns always the middle element
 * after sorting ((\e window_size / 2) + 1) elements. The size of the window 
 * \e window_size can be set in the constructor or with 
 * \e setWindowSize() .
 *
 */
class ALVAR_EXPORT FilterMedian : public FilterAverage {
	std::vector<double> sort_buffer;
public:
	FilterMedian(int size=3) { setWindowSize(size); }
	void setWindowSize(int size) { 
		FilterAverage::setWindowSize(size);
		sort_buffer.resize(size);
	}
	double operator= (double _value) { return next(_value); }
	virtual double next(double y);
};

/**
 * \brief \e FilterRunningAverage provides an weighted running average filter
 * \note   This could be named also as FilterSingleExponentialSmoothing
 *
 * The \e FilterRunningAverage calculates a simple running average
 * using the weight value \e alpha. 
 * \code
 *		value = ((1.0-alpha) * value) + (alpha * (double)y);
 * \endcode
 * If alpha is larger (near 1.0) the average reacts faster for 
 * changes and if it is near 0.0 then it reacts slowly. The 
 * weight value \e alpha may be set in the constructor or 
 * with \e setAlpha() .
 */
class ALVAR_EXPORT FilterRunningAverage : public Filter {
protected:
	double alpha;
	bool breset;
public:
	FilterRunningAverage(double _alpha=0.5) { breset=true; setAlpha(_alpha); }
	void setAlpha(double _alpha) { alpha=std::max(std::min(_alpha,1.0),0.0); }
	double getAlpha() { return alpha; }
	double operator= (double _value) { return next(_value); }
	virtual double next(double y);
	virtual void reset();
};

/**
 * \brief \e FilterDoubleExponentialSmoothing provides an weighted running average filter
 *
 * The \e FilterDoubleExponentialSmoothing calculates a simple running average
 * for both the \e average and \e slope using the weight values \e alpha and
 * \e gamma. 
 * \code
 *		value = ((1.0-alpha) * (value + slope)) + (alpha * (double)y);
 *		slope = ((1.0-gamma) * (slope)) + (gamma * (value - value_prev));
 * \endcode
 * If the weight values (\e alpha , \e gamma) are larger (near 1.0) 
 * the formulas react faster for changes and if they are near 0.0 
 * then the reaction is slower. The weight values \e alpha and \e gamma
 * may be set in the constructor or with \e setAlpha() and \e setGamma() .
 */
class ALVAR_EXPORT FilterDoubleExponentialSmoothing : public FilterRunningAverage {
protected:
	double gamma;
	double slope;
public:
	FilterDoubleExponentialSmoothing(double _alpha=0.5, double _gamma=1.0) : FilterRunningAverage(_alpha) {
		setGamma(_gamma);
	}
	void setGamma(double _gamma) { gamma=std::max(std::min(_gamma,1.0),0.0); }
	double getGamma() { return gamma; }
	double operator= (double _value) { return next(_value); }
	virtual double next(double y);
};

/**
 * \brief Class for handling an array of filtered values
 *
 */
template <class F>
class ALVAR_EXPORT FilterArray {
protected:
	double *tmp;
	std::vector<F> arr;
public:
	FilterArray(int size) {
		tmp = NULL;
		SetSize(size);
	}
	~FilterArray() {
		delete [] tmp;
	}
	size_t GetSize() {
		return arr.size();
	}
	void SetSize(size_t size) {
		if (tmp) delete [] tmp;
		tmp = new double[size];
		arr.resize(size);
	}
	F &operator[](size_t i) {
		return arr[i];
	}
	const double *as_double_array(size_t start_i=0) {
		for (size_t i=0; i<arr.size(); i++) {
			tmp[i] = arr[i];
		}
		return &(tmp[start_i]);
	}
};

} // namespace alvar

#endif

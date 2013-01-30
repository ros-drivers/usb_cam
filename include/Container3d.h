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

#ifndef CONTAINER_3D
#define CONTAINER_3D

/**
 * \file Container3d.h
 *
 * \brief This file implements a generic container to store data in 3D.
 */

#include "cv.h"
#include <utility>
#include <vector>
#include <algorithm>

namespace alvar {

template <class T> class Container3d;

/** \brief Functor class for \e Container3d \e Sort() to sort the search base using distance to specified origin. */
template <class T>
class Container3dSortDist {
protected:
	CvPoint3D32f orig;
	Container3d<T> &container;
public:
	Container3dSortDist(Container3d<T> &_container, const CvPoint3D32f _orig) : container(_container), orig(_orig) {}
	bool operator()(size_t i1, size_t i2)
	{
		float x1 = container[i1].first.x-orig.x, x2 = container[i2].first.x-orig.x;
		float y1 = container[i1].first.y-orig.y, y2 = container[i2].first.y-orig.y;
		float z1 = container[i1].first.z-orig.z, z2 = container[i2].first.z-orig.z;
		float d1 = x1*x1 + y1*y1 + z1*z1;
		float d2 = x2*x2 + y2*y2 + z2*z2;
		return d1<d2;
	}
};

/** \brief Functor class for \e Container3d \e Sort() to sort the search base using content size. */
template <class T>
class Container3dSortSize {
protected:
	Container3d<T> &container;
public:
	Container3dSortSize(Container3d<T> &_container) : container(_container) {}
	bool operator()(size_t i1, size_t i2)
	{
		return (container[i1].second->size() > container[i2].second->size());
	}
};

/** \brief Functor class for \e Container3d \e Limit() to limit the search space with distance */
template <class T>
class Container3dLimitDist {
protected:
	Container3d<T> &container;
	CvPoint3D32f orig;
	float dist_limit;
public:
	Container3dLimitDist(Container3d<T> &_container, const CvPoint3D32f _orig, float _dist_limit) 
		: container(_container),orig(_orig),dist_limit(_dist_limit) {}
	bool operator()(size_t i) const {
		float x = container[i].first.x-orig.x;
		float y = container[i].first.y-orig.y;
		float z = container[i].first.z-orig.z;
		float d = x*x + y*y + z*z;
		if (d <= dist_limit*dist_limit) return true;
		return false;
	}
};

/**
 * \brief Generic container to store any information in 3D (features, photos, ...)
 *
 * You can store any information in 3D using this container. Each element in the container has 
 * an unique id that it can be referenced with using \e operator[](). The indices are from 0
 * to \e size(). You can find specific index using \e GetIndex(Iterator &iter) or \e GetIndex(T *p) .
 *
 * In addition the Container3d contains also a 'search space' that can be iterated through
 * using \e begin() and \e end(). This 'search space' can be limited using \e Limit() , 
 * sorted using \e Sort() and reseted using \e ResetSearchSpace(). You specify what to limit/sort
 * using specified functors. In ALVAR there exists functors \e Container3dLimitDist ,
 * \e Container3dSortSize and \e Container3dSortDist . But you can quite well make your own
 * versions (see example below).
 * 
 * The implementation is optimized for a situation where there are a lot of searches between every 
 * time the search space is limited/ordered. This is the reason we use vector containers
 * internally. The idea is that later we will improve the class by providing more ways for limiting 
 * and sorting the search space; for example Frustum culling.
 *
 * Usage:
 *
 * \code
 *
 * template <class T>
 * class Container3dSortX {
 * protected:
 * 	Container3d<T> &container;
 * public:
 * 	Container3dSortX(Container3d<T> &_container) : container(_container) {}
 * 	bool operator()(size_t i1, size_t i2) const {
 * 		return (container[i1].first.x < container[i2].first.x);
 * 	}
 * };
 * template <class T>
 * class Container3dLimitX {
 * protected:
 * 	int x_min, x_max;
 * 	Container3d<T> &container;
 * public:
 * 	Container3dLimitX(Container3d<T> &_container, int _x_min, int _x_max) 
 * 	: container(_container),x_min(_x_min),x_max(_x_max) {}
 * 	bool operator()(size_t i1) const {
 * 		if ((container[i1].first.x >= x_min) && (container[i1].first.x <= x_max)) return true;
 * 		return false;
 * 	}
 * };
 * 
 * ...
 * Container3d<int> c3d;
 * c3d.Add(CvPoint3D32f(0,0,0), 0);
 * c3d.Add(CvPoint3D32f(5,0,0), 1);
 * c3d.Add(CvPoint3D32f(0,5,0), 2);
 * c3d.Add(CvPoint3D32f(0,0,5), 3);
 * c3d.Add(CvPoint3D32f(0,0,500), 4);
 * c3d.Add(CvPoint3D32f(500,0,0), 5);
 * c3d.Add(CvPoint3D32f(1,0,0), 6);
 * c3d.Add(CvPoint3D32f(0,0,1), 7);
 * Container3dSortX<int> sortx(c3d);
 * Container3dLimitX<int> limitx(c3d, -10, 10);
 * Container3dLimitDist<int> limit_dist(c3d, cvPoint3D32f(0,0,0), 10.0);
 * c3d.ResetSearchSpace();                // Search space: 0,1,2,3,4,5,6,7
 * c3d.Sort(sortx);                       // Search space: 0,2,3,4,7,6,1,5
 * c3d.Limit(limitx);                     // Search space: 0,2,3,4,7,6,1
 * c3d.Limit(limit_dist);                 // Search space: 0,2,3,7,6,1
 * Container3d<int>::Iterator iter;
 * for (iter=c3d.begin(); iter != c3d.end(); ++iter) {
 *     cout<<" "<<iter->second;
 * }
 * \endcode
 *
 */
template <class T>
class Container3d
{
	public:
		/** \brief \e node_type for storing data. 3D-position is paired with the data content. */
		typedef std::pair<CvPoint3D32f, T> node_type;
	protected:
		/** \brief the actual data in using node_type: pair<CvPoint3D32f, T> */
		std::vector<node_type> data;
		/** \brief Possibly limited set of indices for \e data in somehow "optimal" search order */
		std::vector<size_t>    search_space;

	public:
		/** \brief Add \e _data in the container and associate it with 3D position \e _pos */
		void Add(const CvPoint3D32f& _pos, const T& _data){
			data.push_back(node_type(_pos, _data));
			search_space.push_back(data.size()-1);
		}
		/** \brief Clear the container */
		void Clear() {
			data.clear();
			search_space.clear();
		}
		/** \brief Reset the search space to contain whole data */
		void ResetSearchSpace() {
			search_space.resize(data.size());
			for (size_t i=0; i<search_space.size(); i++)
			{
				search_space[i] = i;
			}
		}
		/** \brief Erase item in the container */
		void Erase(size_t index) {
			typename std::vector<node_type>::iterator iter_d;
			iter_d = data.begin();
			for (size_t i=0; i<index; i++) iter_d++;
			data.erase(iter_d);
			ResetSearchSpace();
		}
		/** \brief Sort using external Compare method */
		template <typename Compare>
		int Sort(Compare comp) {
			stable_sort(search_space.begin(), search_space.end(), comp);
			return search_space.size();
		}

		/** \brief Limit the search space with external limitation */
		template <typename Test>
		int Limit(Test test) {
			std::vector<size_t>::iterator iter;
			for (iter = search_space.begin(); iter != search_space.end();) {
				if (!test(*iter)) {
					iter = search_space.erase(iter);
				} else {
					iter++;
				}
			}
			return search_space.size();
		}

		/** \brief Iterator for going through the items in \e Container3d in the specified order
		 *
		 * The idea is that the content in \e Container3d can be sorted and limited in different
		 * ways. After sorting/limiting the content the \e iterator (\e Begin() and \e End() ) can
		 * be used for accessing the data items in optimal order.
		 */
		class Iterator : public std::iterator<std::forward_iterator_tag, node_type>
		{
			protected:
				Container3d<T> *container;
				std::vector<size_t>::iterator iter;
			public:
				Iterator() {}
				Iterator(Container3d<T> *_container, std::vector<size_t>::iterator _iter) : container(_container), iter(_iter) {}
				node_type &operator*() const { return container->data[*iter]; }
				node_type *operator->() const { return &(operator*()); }
				virtual Iterator& operator++() { ++iter; return *this; }
				bool operator==(const Iterator& _m) const { return iter == _m.iter; }
				bool operator!=(const Iterator& _m) const { return iter != _m.iter; }
				size_t GetIndex() { return *iter; }
		};

		/** \brief Provides an iterator pointing to the beginning of the limited/sorted 3D content */
		Iterator begin() {
			return Iterator(this, search_space.begin());
		}

		/** \brief Provides an iterator pointing to the end of the limited/sorted 3D content */
		Iterator end() {
			return Iterator(this, search_space.end());
		}

		/** \brief Get number of items that can be referenced using \e operator[]() */
		size_t size() const { return data.size(); }

		/** \brief Get absolute reference usable with \e operator[]() based on the iterator */
		size_t GetIndex(Iterator &iter) {
			return iter.GetIndex();
		}

		/** \brief Instead of \e Iterator we can use also absolute references for data with \e operator[]() */
		node_type &operator[](size_t index) {
			return data[index];
		}

		/** \brief Get absolute reference usable with \e operator[]() based on the content */
		size_t GetIndex(T *p) {
			size_t i=0;
			for (; i<data.size(); ++i)
			{
				if (data[i].second.get() == p) break;
			}
			return i;
		}
};

} // namespace alvar

#endif


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

#include "Ransac.h"
#include <memory.h>
#include <math.h>

#ifdef TRACE
    #include <stdio.h>
#endif

namespace alvar {

RansacImpl::RansacImpl(int min_params, int max_params, 
                       int sizeof_param, int sizeof_model) {
  this->min_params = min_params;
  this->max_params = max_params;
  this->sizeof_param = sizeof_param;
  this->sizeof_model = sizeof_model;
  indices = NULL;

  samples = new void*[max_params];
  if (!samples) {
#ifdef TRACE
    printf("Cound not allocate memory for %d sample pointers.",
		  max_params);
#endif
  }

  hypothesis = new char[sizeof_model];
  if (!hypothesis) {
#ifdef TRACE
    printf("Cound not allocate %d bytes of memory for model parameters.",
		  sizeof_model);
#endif
  }
}

RansacImpl::~RansacImpl() {
  if (samples) delete[] samples;
  if (hypothesis) delete[] (char *)hypothesis;
  if (indices) delete[] indices;
}

int RansacImpl::_estimate(void* params, int param_c,
			  int support_limit, int max_rounds,
			  void* model) {
            if (param_c < min_params) return 0;

            int max_support = 0;

            // Randomly search for the best model.
            for (int i = 0; 
                 i < max_rounds && max_support < support_limit; 
                 i++) {
                // 1. pick a random sample of min_params.
                int sample_c;
                for (sample_c = 0; sample_c < min_params; sample_c++) {
                    int r = rand() % (param_c-sample_c);
                    void* s = (char*)params + r*sizeof_param;
                    for (int j = 0; j < sample_c; j++) 
                        if (s >= samples[j]) s = (char*)s + sizeof_param;
                    samples[sample_c] = s;
                }

                // 2. create a model from the sampled parameters.
                _doEstimate(samples, sample_c, hypothesis);

                // 3. count the support for the model.
                int hypo_support = 0;
                for (int j = 0; j < param_c; j++) {
                    if (_doSupports((char*)params + j*sizeof_param, hypothesis)) {
                        hypo_support++;
                    }
                }
#ifdef TRACE
                printf("Hypothesis got %d support\n", hypo_support);
#endif
                if (hypo_support > max_support) {
                    max_support = hypo_support;
                    memcpy(model, hypothesis, sizeof_model);
                }
            }

            return max_support;
}

int RansacImpl::_refine(void* params, int param_c,
			int support_limit, int max_rounds,
			void* model, char *inlier_mask) {
  if (param_c < min_params) return 0;

  int max_support = 0;

  // Iteratively make the model estimation better.
  for (int i = 0; 
       i < max_rounds && max_support < support_limit; 
       i++) {
    int sample_c = 0;
    // 1. Find all parameters that support the current model.
    for (int j = 0; j < param_c && sample_c < max_params; j++) {
      if (_doSupports((char*)params + j*sizeof_param, model)) {
	      samples[sample_c++] = (char*)params + j*sizeof_param;
        if (inlier_mask) inlier_mask[j] = 1;
      } else {
        if (inlier_mask) inlier_mask[j] = 0;
      }
    }
#ifdef TRACE
    printf("Found %d supporting parameters\n", sample_c);
#endif
    if (sample_c > max_support) {
      // 2. create a model from all supporting parameters.
      _doEstimate(samples, sample_c, model);
      max_support = sample_c;
      
    } else {
      // until there are no new supporting parameters.
      break;
    }
  }
  
  return max_support;
}

/** IndexRansac version */

RansacImpl::RansacImpl(int min_params, int max_params, int sizeof_model) {
  this->min_params = min_params;
  this->max_params = max_params;
  this->sizeof_param = -1;
  this->sizeof_model = sizeof_model;
  samples = NULL;
  indices = new int[max_params];
  hypothesis = new char[sizeof_model];
  if (!hypothesis) {
#ifdef TRACE
    printf("Cound not allocate %d bytes of memory for model parameters.",
		  sizeof_model);
#endif
  }
}

int RansacImpl::_estimate(int param_c,
		                      int support_limit, int max_rounds,
                          void* model) {
  if (param_c < min_params) return 0;

  int max_support = 0;

  // Randomly search for the best model.
  for (int i = 0; 
       i < max_rounds && max_support < support_limit; 
       i++) {
      // 1. pick a random sample of min_params.
      int sample_c;
      for (sample_c = 0; sample_c < min_params; sample_c++) {
          int r = rand() % (param_c-sample_c);
          for (int j = 0; j < sample_c; j++) 
              if (r >= indices[j]) r++;
          indices[sample_c] = r;
      }

      // 2. create a model from the sampled parameters.
      _doEstimate(indices, sample_c, hypothesis);

      // 3. count the support for the model.
      int hypo_support = 0;
      for (int j = 0; j < param_c; j++) {
          if (_doSupports(j, hypothesis)) {
              hypo_support++;
          }
      }
#ifdef TRACE
      printf("Hypothesis got %d support\n", hypo_support);
#endif
      if (hypo_support > max_support) {
          max_support = hypo_support;
          memcpy(model, hypothesis, sizeof_model);
      }
  }

  return max_support;
}

int RansacImpl::_refine(int param_c,
	                      int support_limit, int max_rounds,
                        void* model, char *inlier_mask) {
  if (param_c < min_params) return 0;

  int max_support = 0;

  // Iteratively make the model estimation better.
  for (int i = 0; 
       i < max_rounds && max_support < support_limit; 
       i++) {
    int sample_c = 0;
    // 1. Find all parameters that support the current model.
    for (int j = 0; j < param_c && sample_c < max_params; j++) {
      if (_doSupports(j, model)) {
	      indices[sample_c++] = j;
        if (inlier_mask) inlier_mask[j] = 1;
      } else {
        if (inlier_mask) inlier_mask[j] = 0;
      }
    }
#ifdef TRACE
    printf("Found %d supporting parameters\n", sample_c);
#endif
    if (sample_c < min_params) break; // indicates too few points.
    if (sample_c > max_support) {
      // 2. create a model from all supporting parameters.
      _doEstimate(indices, sample_c, model);
      max_support = sample_c;
      
    } else {
      // until there are no new supporting parameters.
      break;
    }
  }
  
  return max_support;
}

/** public methods */

int RansacImpl::estimateRequiredRounds(float success_propability,
				       float inlier_percentage) {
  return (int) 
    (log(1-success_propability) / log(1-pow(inlier_percentage,3)));
}

} // namespace alvar

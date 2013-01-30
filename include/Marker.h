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

#ifndef MARKER_H
#define MARKER_H

/**
 * \file Marker.h
 *
 * \brief This file implements a marker interface as well as ALVAR markers and
 * ARToolKit markers.
 */

#include "Alvar.h"
#include <iostream>
#include <algorithm>
#include "Util.h"
#include "Camera.h"
#include "Pose.h"
#include "Bitset.h"
#include <vector>
#include "ar_track_alvar/kinect_filtering.h"

namespace alvar {

  /**
   * \brief Basic 2D \e Marker functionality.
   *
   * This class contains the basic \e Marker functionality for planar markers.
   */
  class ALVAR_EXPORT Marker
  {
  protected:
    void VisualizeMarkerPose(IplImage *image, Camera *cam, double visualize2d_points[12][2], CvScalar color=CV_RGB(255,0,0)) const;
    virtual void VisualizeMarkerContent(IplImage *image, Camera *cam, double datatext_point[2], double content_point[2]) const;
    virtual void VisualizeMarkerError(IplImage *image, Camera *cam, double errortext_point[2]) const;
    bool UpdateContentBasic(std::vector<Point<CvPoint2D64f> > &_marker_corners_img, IplImage *gray, Camera *cam, int frame_no = 0);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    bool valid;

    /** \brief Compares the marker corners with the previous match. 
     *
     *  In some cases the tracking of the marker can be accepted solely based on this.
     *  Returns the marker orientation and an error value describing the pixel error 
     *  relative to the marker diameter.
     */
    void CompareCorners(std::vector<Point<CvPoint2D64f> > &_marker_corners_img, int *orientation, double *error);
    /** \brief Compares the marker corners with the previous match. 
     */
    void CompareContent(std::vector<Point<CvPoint2D64f> > &_marker_corners_img, IplImage *gray, Camera *cam, int *orientation) const;
    /** \brief Updates the \e marker_content from the image using \e Homography
     */
    virtual bool UpdateContent(std::vector<Point<CvPoint2D64f> > &_marker_corners_img, IplImage *gray, Camera *cam, int frame_no = 0);
    /** \brief Updates the markers \e pose estimation
     */
    void UpdatePose(std::vector<Point<CvPoint2D64f> > &_marker_corners_img, Camera *cam, int orientation, int frame_no = 0, bool update_pose = true);
    /** \brief Decodes the marker content. Please call \e UpdateContent before this. 
     *  This virtual method is meant to be implemented by heirs.
     */
    virtual bool DecodeContent(int *orientation);
	
    /** \brief Returns the content as a matrix
     */
    CvMat *GetContent() const {
      return marker_content;
    }
    /** \brief Saves the marker as an image
     */
    void SaveMarkerImage(const char *filename, int save_res = 0) const;
    /** \brief Draw the marker filling the ROI in the given image
     */
    void ScaleMarkerToImage(IplImage *image) const;
    /** \brief Visualize the marker
     */
    void Visualize(IplImage *image, Camera *cam, CvScalar color=CV_RGB(255,0,0)) const;
    /** \brief Method for resizing the marker dimensions  */
    void SetMarkerSize(double _edge_length = 0, int _res = 0, double _margin = 0);
    /** \brief Get edge length (to support different size markers */
    double GetMarkerEdgeLength() const { return edge_length; }
    /** \brief Destructor  */
    ~Marker();
    /** \brief Default constructor 
     * \param _edge_length Length of the marker's edge in whatever units you are using (e.g. cm)
     * \param _res The marker content resolution in pixels (this is actually 
     * \param _margin The marker margin resolution in pixels (The actual captured marker image has pixel resolution of _margin+_res+_margin)
     */
    Marker(double _edge_length = 0, int _res = 0, double _margin = 0);
    /** \brief Copy constructor  */
    Marker(const Marker& m);
    /** \brief Get id for this marker
     * This is used e.g. in MarkerDetector to associate a marker id with
     * an appropriate edge length. This method should be overwritten
     * to return a suitable identification number for each marker type.
     */
    virtual unsigned long GetId() const { return 0; }
    virtual void SetId(unsigned long _id) {};

    /**
     * Returns the resolution (the number of square rows and columns) of the 
     * marker content area. The total number of content squares within the
     * content area is resolution*resolution.
     */
    int GetRes() const { return res; }

    /**
     * Returns the margin thickness, that is, the number of rows or columns of
     * black squares surrounding the content area.
     */
    double GetMargin() const { return margin; }

    /** \brief The current marker \e Pose
     */
    Pose pose;
    /** \brief Get marker detection error estimate
     * \param errors Flags indicating what error elements are combined
     * The marker detection error can consist of several elements:
     * MARGIN_ERROR is updated in \e UpdateContent and it indicates erroneous values inside the marginal area.
     * DECODE_ERROR is updated in  \e DecodeContent and it indicates erroneous values inside the actual marker content area.
     * TRACK_ERROR is updated in  \e MarkerDetector.Detect and it indicates the amount of tracking error returned from \e CompareCorners
     */
    double GetError(int errors = (MARGIN_ERROR | DECODE_ERROR)) const {
      int count = 0;
      double error = 0;
      if (errors & MARGIN_ERROR) {error+=margin_error; count++;}
      if (errors & DECODE_ERROR) {error+=decode_error; count++;}
      if (errors & TRACK_ERROR) {error+=track_error; count++;}
      return error/count;
    }
    /** \brief Set the marker error estimate */
    void SetError(int error_type, double value) {
      if (error_type == MARGIN_ERROR) margin_error = value;
      else if (error_type == DECODE_ERROR) decode_error = value;
      else if (error_type == TRACK_ERROR) track_error = value;
    }
    static const int MARGIN_ERROR=1;
    static const int DECODE_ERROR=2;
    static const int TRACK_ERROR=4;
  protected:
    double margin_error;
    double decode_error;
    double track_error;
    double edge_length;
    int res;
    double margin;
    CvMat *marker_content;

  public:
      
    /** \brief Marker color points in marker coordinates */
    std::vector<PointDouble> marker_points;
    /** \brief Marker corners in marker coordinates */
    std::vector<PointDouble> marker_corners;
    /** \brief Marker corners in image coordinates */
    std::vector<PointDouble> marker_corners_img;
    /** \brief Marker points in image coordinates */
    std::vector<PointDouble> ros_marker_points_img;
    ar_track_alvar::ARCloud ros_corners_3D;
    int ros_orientation;
    /** \brief Samples to be used in figuring out min/max for thresholding */
    std::vector<PointDouble> marker_margin_w;
    /** \brief Samples to be used in figuring out min/max for thresholding */
    std::vector<PointDouble> marker_margin_b;
#ifdef VISUALIZE_MARKER_POINTS
    std::vector<PointDouble> marker_allpoints_img;
#endif
  };

  /**
   * \brief \e MarkerArtoolkit for using matrix markers similar with the ones used in ARToolkit.
   */
  class ALVAR_EXPORT MarkerArtoolkit : public Marker
  {
  protected:
    int default_res() { std::cout<<"MarkerArtoolkit::default_res"<<std::endl; return 3; }
    double default_margin() { return 1.5; }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    /** \brief \e MarkerArtoolkit supports only 'id' as data type */
    unsigned long id;
    /** \brief Constructor */
  MarkerArtoolkit(double _edge_length = 0, int _res = 0, double _margin = 0) : 
    Marker(_edge_length, (_res?_res:3), (_margin?_margin:1.5))
      {
      }
    /** \brief Get ID for recognizing this marker */
    unsigned long GetId() const { return id; }
    void SetId(unsigned long _id) { id = _id; }
    /** \brief \e DecodeContent should be called after \e UpdateContent to fill \e content_type, \e decode_error and \e data */
    bool DecodeContent(int *orientation);
    /** \brief Updates the \e marker_content by "encoding" the given parameters */
    void SetContent(unsigned long _id);
  };

  /**
   * \brief \e MarkerData contains matrix of Hamming encoded data.
   */
  class ALVAR_EXPORT MarkerData : public Marker
  {
  protected:
    virtual void VisualizeMarkerContent(IplImage *image, Camera *cam, double datatext_point[2], double content_point[2]) const;
    void DecodeOrientation(int *error, int *total, int *orientation);
    int DecodeCode(int orientation, BitsetExt *bs, int *erroneous, int *total, unsigned char* content_type);
    void Read6bitStr(BitsetExt *bs, char *s, size_t s_max_len);
    void Add6bitStr(BitsetExt *bs, char *s);
    int UsableDataBits(int marker_res, int hamming);
    bool DetectResolution(std::vector<Point<CvPoint2D64f> > &_marker_corners_img, IplImage *gray, Camera *cam);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    static const int MAX_MARKER_STRING_LEN=2048;
    enum MarkerContentType {
      MARKER_CONTENT_TYPE_NUMBER,
      MARKER_CONTENT_TYPE_STRING,
      MARKER_CONTENT_TYPE_FILE,
      MARKER_CONTENT_TYPE_HTTP
    };
    unsigned char	content_type;

    /** \brief \e MarkerData content can be presented either as number (\e MARKER_CONTENT_TYPE_NUMBER) or string */
    union {
      unsigned long	id;							
      char			str[MAX_MARKER_STRING_LEN]; 
    } data;

    /** \brief Default constructor 
     * \param _edge_length Length of the marker's edge in whatever units you are using (e.g. cm)
     * \param _res The marker content resolution in pixels (this is actually 
     * \param _margin The marker margin resolution in pixels (The actual captured marker image has pixel resolution of _margin+_res+_margin)
     */
  MarkerData(double _edge_length = 0, int _res = 0, double _margin = 0) : 
    Marker(_edge_length, _res, (_margin?_margin:2))
      {
      }
    /** \brief Get ID for recognizing this marker */
    unsigned long GetId() const { return data.id; }
    /** \brief Set the  ID */
    void SetId(unsigned long _id) { data.id = _id; }
    /** \brief Updates the \e marker_content from the image using \e Homography
     * Compared to the basic implementation in \e Marker this will also detect the marker 
     * resolution automatically when the marker resolution is specified to be 0.
     */
    virtual bool UpdateContent(std::vector<Point<CvPoint2D64f> > &_marker_corners_img, IplImage *gray, Camera *cam, int frame_no = 0);
    /** \brief \e DecodeContent should be called after \e UpdateContent to fill \e content_type, \e decode_error and \e data 
     */
    bool DecodeContent(int *orientation);
    /** \brief Updates the \e marker_content by "encoding" the given parameters
     */
    void SetContent(MarkerContentType content_type, unsigned long id, const char *str, bool force_strong_hamming=false, bool verbose=false);
  };

  /** \brief Iterator type for traversing templated Marker vector without the template.
   */
  class ALVAR_EXPORT MarkerIterator : public std::iterator<std::forward_iterator_tag, Marker*> {
  public:
    virtual bool operator==(const MarkerIterator& other) = 0;
    virtual bool operator!=(const MarkerIterator& other) = 0;
    virtual MarkerIterator& operator++() = 0;
    virtual const Marker* operator*() = 0;
    virtual const Marker* operator->() = 0;
    virtual MarkerIterator& reset() = 0;

    void *_data;
  };

  /** \brief Iterator implementation for traversing templated Marker vector
   * without the template.
   * \param T T extends Marker
   */
  template<typename T>
    class ALVAR_EXPORT MarkerIteratorImpl : public MarkerIterator {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
  MarkerIteratorImpl(typename std::vector<T>::const_iterator i) : _begin(i), _i(i) {
      _data = this;
    }

    ~MarkerIteratorImpl() {}

    // The assignment and relational operators are straightforward
    MarkerIteratorImpl& operator=(const MarkerIteratorImpl& other) {
      _i = other._i;
      return(*this);
    }

    bool operator==(const MarkerIterator& other) {
      MarkerIteratorImpl* pother = (MarkerIteratorImpl*)other._data;
      return (_i == pother->_i);
    }

    bool operator!=(const MarkerIterator& other) {
      MarkerIteratorImpl* pother = (MarkerIteratorImpl*)other._data;
      return (_i != pother->_i);
    }

    MarkerIterator& operator++() {
      _i++;
      return(*this);
    }

    const Marker* operator*() {
      return &(*_i);
    }

    const Marker* operator->() {
      return &(*_i);
    }

    MarkerIterator& reset() {
      _i = _begin;
      return (*this);
    }

  private:
    typename std::vector<T>::const_iterator _begin;
    typename std::vector<T>::const_iterator _i;
  };

} // namespace alvar

#endif

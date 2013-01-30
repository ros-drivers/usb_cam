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

#ifndef ALVAR_H
#define ALVAR_H

/**
 * \mainpage ALVAR ${ALVAR_VERSION}
 *
 * \section Introduction
 *
 * ALVAR is a software library for creating virtual and augmented reality (AR) applications. ALVAR has
 * been developed by the VTT Technical Research Centre of Finland. ALVAR is released under the terms of
 * the GNU Lesser General Public License, version 2.1, or (at your option) any later version.
 *
 * ALVAR is designed to be as flexible as possible. It offers high-level tools and methods for creating
 * augmented reality applications with just a few lines of code. The library also includes interfaces
 * for all of the low-level tools and methods, which makes it possible for the user to develop their
 * own solutions using alternative approaches or completely new algorithms.
 *
 * ALVAR is currently provided on Windows and Linux operating systems and only depends on one third
 * party library (OpenCV). ALVAR is independent of any graphical libraries and can be easily integrated
 * into existing applications. The sample applications use GLUT and the demo applications use OpenSceneGraph.
 *
 * \section Features
 *
 * - Detecting and tracking 2D markers (\e MarkerDetector). Currently two types of square matrix markers
 *   are supported (\e MarkerData and \e MarkerArtoolkit). Future marker types can easily be added. ALVAR
 *   keeps the \e Marker \e Pose estimation as accurate as possible. Furthermore, ALVAR uses some tracking
 *   heuristics to identify markers that are "too far" and to recover from occlusions in the multimarker
 *   case for example.
 * - Using a setup of multiple markers for pose detection (\e MultiMarker). The marker setup coordinates
 *   can be set manually or they can be automatically deduced using various methods (\e MultiMarkerFiltered
 *   and \e MultiMarkerBundle).
 * - Tools for calibrating \e Camera. Distorting and undistorting points, projecting points and finding
 *   exterior orientation using point-sets.
 * - Hiding markers from the view (\e BuildHideTexture and \e DrawTexture).
 * - Several basic filters: \e FilterAverage, \e FilterMedian, \e FilterRunningAverage,
 *   \e FilterDoubleExponentialSmoothing.
 * - \e Kalman filters for sensor fusion: \e Kalman Filter, \e Extended Kalman Filter and Unscented Kalman
 *   Filter (\e KalmanSensor, \e KalmanSensorEkf, \e KalmanEkf, \e UnscentedKalman).
 * - Several methods for tracking using optical flow: \e TrackerPsa , \e TrackerPsaRot , \e TrackerFeatures
 *   and \e TrackerStat.
 * - etc...
 *
 * \section Platforms
 *
 * ALVAR is officially supported and tested on the following platforms.
 *  - Windows XP 32-bit, Microsoft Visual Studio 2005 (8.0), 2008 (9.0) and 2010 (10.0)
 *  - Linux 32-bit, GCC 4.3 and 4.4
 *  - Linux 64-bit, GCC 4.3 and 4.4
 *
 * \section Dependencies
 *
 * The ALVAR library depends on the following libraries.
 *  - OpenCV 2.4.0
 *
 * The ALVAR samples depend on the following libraries and tools.
 *  - GLUT 3.7.6
 *  - CMake 2.8.3
 *
 * The ALVAR demos depend on the following libraries and tools.
 *  - OpenSceneGraph 2.8.4
 *  - CMake 2.8.3
 *
 * \section Usage
 *
 * Please see the instructions in doc/Compiling.txt for more information.
 *
 * \section Links
 *
 * - ALVAR Development Team at VTT (http://www.vtt.fi/multimedia)
 * - CMake (http://www.cmake.org)
 * - OpenCV -- Open Computer Vision Library (http://code.opencv.org)
 * - GLUT Library (http://www.opengl.org/resources/libraries/glut)
 * - OpenSceneGraph (http://www.openscenegraph.org)
 *
 * \example SampleCamCalib.cpp
 * This is an example of how to use \e ProjPoints and \e Camera classes to perform camera calibration
 * using a chessboard pattern.
 *
 * \example SampleCvTestbed.cpp
 * This is an example of how to use the \e CvTestbed and \e Capture classes in order to make quick OpenCV
 * prototype applications.
 *
 * \example SampleFilter.cpp
 * This is an example of how to use various filters: \e FilterAverage, \e FilterMedian,
 * \e FilterRunningAverage, \e FilterDoubleExponentialSmoothing and \e Kalman.
 *
 * \example SampleIntegralImage.cpp
 * This is an example of how to use the \e IntegralImage and \e IntegralGradient classes for image
 * gradient analysis.
 *
 * \example SampleLabeling.cpp
 * This is an example of how to label images using \e LabelImage and \e MarchEdge.
 *
 * \example SampleMarkerCreator.cpp
 * This is an example that demonstrates the generation of \e MarkerData markers and saving the image
 * using \e SaveMarkerImage.
 *
 * \example SampleMarkerDetector.cpp
 * This is an example that shows how to detect \e MarkerData markers and visualize them using\e GlutViewer.
 *
 * \example SampleMarkerHide.cpp
 * This is an example that shows how to detect \e MarkerData markers, visualize them using \e GlutViewer
 * and hide them with \e BuildHideTexture and \e DrawTexture.
 *
 * \example SampleMarkerlessCreator.cpp
 * This is an example that demonstrates the use of FernImageDetector to train a Fern classifier.
 *
 * \example SampleMarkerlessDetector.cpp
 * This is an example that demonstrates the use of FernImageDetector to detect an image as a marker.
 *
 * \example SampleMultiMarker.cpp
 * This is an example that demonstrates the use of a preconfigured \e MultiMarker setup.
 *
 * \example SampleMultiMarkerBundle.cpp
 * This is an example that automatically recognising \e MultiMarker setups using \e MultiMarkerFiltered and
 * optimizes it with \e MultiMarkerBundle.
 *
 * \example SampleOptimization.cpp
 * This is an example of how to use the \e Optimization class by fitting curves of increasing degree to
 * random data.
 *
 * \example SamplePointcloud.cpp
 * This is an example showing how to use \e SimpleSfM for tracking the environment using features in
 * addition to \e MultiMarker.
 *
 * \example SampleTrack.cpp
 * This is an example that shows how to perform tracking of the optical flow using \e TrackerPsa,
 * \e TrackerPsaRot, \e TrackerFeatures or \e TrackerStat.
 */
 
/**
 * \file Alvar.h
 *
 * \brief This file defines library export definitions, version numbers and
 * build information.
 */

#if defined(WIN32) && !defined(ALVAR_STATIC)
    #ifdef ALVAR_BUILD
        #define ALVAR_EXPORT __declspec(dllexport)
    #else
        #define ALVAR_EXPORT __declspec(dllimport)
    #endif
#else
	#define ALVAR_EXPORT
#endif

/**
 * \brief Main ALVAR namespace.
 */
namespace alvar {

/**
 * \brief Major version number.
 */
static const int ALVAR_VERSION_MAJOR = ${ALVAR_VERSION_MAJOR};

/**
 * \brief Minor version number.
 */
static const int ALVAR_VERSION_MINOR = ${ALVAR_VERSION_MINOR};

/**
 * \brief Patch version number.
 */
static const int ALVAR_VERSION_PATCH = ${ALVAR_VERSION_PATCH};

/**
 * \brief Tag version string.
 *
 * The tag contains alpha, beta and release candidate versions.
 */
static const char *ALVAR_VERSION_TAG = "${ALVAR_VERSION_TAG}";

/**
 * \brief Revision version string.
 *
 * The revision contains an identifier from the source control system.
 */
static const char *ALVAR_VERSION_REVISION = "${ALVAR_VERSION_REVISION}";

/**
 * \brief Entire version string.
 */
static const char *ALVAR_VERSION = "${ALVAR_VERSION}";

/**
 * \brief Entire version string without dots.
 */
static const char *ALVAR_VERSION_NODOTS = "${ALVAR_VERSION_NODOTS}";

/**
 * \brief Date the library was built.
 */
static const char *ALVAR_DATE = "${ALVAR_DATE}";

/**
 * \brief System the library was built on.
 */
static const char *ALVAR_SYSTEM = "${ALVAR_SYSTEM}";

}

#endif

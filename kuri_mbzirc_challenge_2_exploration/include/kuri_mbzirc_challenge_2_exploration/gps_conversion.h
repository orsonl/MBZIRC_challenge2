/****************************************************************************
 *
 *   Copyright (C) 2012, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Modified version of geo.h from the PX4 firmware (https://github.com/PX4/Firmware/blob/master/src/lib/geo/geo.h):
 *  Converted to class
 *
 *
 *
 * @file geo.h
 *
 * Definition of geo / math functions to perform geodesic calculations
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * Additional functions - @author Doug Weibel <douglas.weibel@colorado.edu>
 */

#ifndef KURI_MBZIRC_CHALLENGE_2_TOOLS_GPS_CONVERSION_H_
#define KURI_MBZIRC_CHALLENGE_2_TOOLS_GPS_CONVERSION_H_

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <float.h>
#include <stdint.h>

#define CONSTANTS_ONE_G					              9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				        287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			  -273.15f		/* °C			*/
#define CONSTANTS_RADIUS_OF_EARTH			        6371000			/* meters (m)		*/

struct GeoPoint
{
  double lat, lon, heading;
};

class GPSHandler
{
private:
  double lat_rad_;
  double lon_rad_;
  double sin_lat_;
  double cos_lat_;
  bool init_done_;
  uint64_t timestamp_;

public:
  /**
   * Checks if global projection was initialized
   * @return true if map was initialized before, false else
   */
  bool isInit();

  /**
   * Initializes the global map transformation.
   *
   * Initializes the transformation between the geographic coordinate system and
   * the azimuthal equidistant plane
   * @param lat in degrees (47.1234567°, not 471234567°)
   * @param lon in degrees (8.1234567°, not 81234567°)
   */
  void update(double lat_0, double lon_0, uint64_t timestamp);

  /**
   * Transforms a point in the geographic coordinate system to the local
   * azimuthal equidistant plane using the global projection
   * @param x north
   * @param y east
   * @param lat in degrees (47.1234567°, not 471234567°)
   * @param lon in degrees (8.1234567°, not 81234567°)
   * @return 0 if init was called before, -1 else
   */
  int projectGPSToCartesian(double lat, double lon, float *x, float *y);

  /**
   * Transforms a point in the local azimuthal equidistant plane to the
   * geographic coordinate system using the global projection
   *
   * @param x north
   * @param y east
   * @param lat in degrees (47.1234567°, not 471234567°)
   * @param lon in degrees (8.1234567°, not 81234567°)
   * @return 0 if map_projection_init was called before, -1 else
   */
  int projectCartesianToGPS(float x, float y, double *lat, double *lon);

  double getLat()
  {
    return lat_rad_ * (180.0/M_PI);
  }

  double getLon()
  {
    return lon_rad_ * (180.0/M_PI);
  }
};


#endif

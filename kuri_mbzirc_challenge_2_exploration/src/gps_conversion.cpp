/****************************************************************************
 * equations taken from geo file in the firmware which uses
   Azimuthal Equidistant Projection formulas according to:
   http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html


 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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

#include <signal.h>
#include <termios.h>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <time.h>
#include <algorithm>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>

#include <kuri_mbzirc_challenge_2_exploration/gps_conversion.h>

//*************************this section defined the functions needed for GPS calculations*****************
//********GPS_begin*******************

void GPSHandler::update(double lat_0, double lon_0, uint64_t timestamp)
{
  lat_rad_ = lat_0 * (M_PI/180.0);
  lon_rad_ = lon_0 * (M_PI/180.0);
  sin_lat_ = sin(lat_rad_);
  cos_lat_ = cos(lat_rad_);

  timestamp_ = timestamp;
  init_done_ = true;
}

bool GPSHandler::isInit()
{
  return init_done_;
}


int GPSHandler::projectGPSToCartesian(double goal_lat, double goal_lon, float *x, float *y)
{
  if (!init_done_)
          return -1;

  //std::setprecision(10);
  double goal_lat_rad = goal_lat * (M_PI/180.0);
  double goal_lon_rad = goal_lon * (M_PI/180.0);

  double goal_sin_lat = sin(goal_lat_rad);
  double goal_cos_lat = cos(goal_lat_rad);
  double cos_d_lon = cos(goal_lon_rad - lon_rad_);

  double arg = sin_lat_ * goal_sin_lat + cos_lat_ * goal_cos_lat * cos_d_lon;

  if (arg > 1.0) {
          arg = 1.0;

  } else if (arg < -1.0) {
          arg = -1.0;
  }

  double c = acos(arg);
  double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

  *x = k * (cos_lat_ * goal_sin_lat - sin_lat_ * goal_cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
  *y = -k * goal_cos_lat * sin(goal_lon_rad - lon_rad_) * CONSTANTS_RADIUS_OF_EARTH;

  return 0;
}


int GPSHandler::projectCartesianToGPS(float x, float y, double *goal_lat, double *goal_lon)
{
  if (!init_done_)
    return -1;

  double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
  double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
  double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
  double sin_c = sin(c);
  double cos_c = cos(c);

  double goal_lat_rad;
  double goal_lon_rad;

  if (fabs(c) > DBL_EPSILON) {
          goal_lat_rad = asin(cos_c * sin_lat_ + (x_rad * sin_c * cos_lat_) / c);
          goal_lon_rad = (lon_rad_ + atan2(y_rad * sin_c, c * cos_lat_ * cos_c - x_rad * sin_lat_ * sin_c));

  } else {
          goal_lat_rad = lat_rad_;
          goal_lon_rad = lon_rad_;
  }

  *goal_lat = goal_lat_rad * (180.0/M_PI);
  *goal_lon = goal_lon_rad * (180.0/M_PI);

  return 0;
}

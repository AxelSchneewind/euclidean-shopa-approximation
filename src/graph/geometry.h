#pragma once

#include "base_types.h"

#include "cmath"

// Euclidean distance
double
distance_euclidean(coordinate_t c1, coordinate_t c2);

inline double
to_radians(double __degree);

// exact distance on earths surface
distance_t
distance(coordinate_t __c1, coordinate_t __c2);

/**
 * angle in radians
 * @param __s0
 * @param __d0
 * @param __s1
 * @param __d1
 * @return
 */
inline double
angle(coordinate_t __s0, coordinate_t __d0, coordinate_t __s1, coordinate_t __d1);

inline double
angle(coordinate_t dir0, coordinate_t dir1);

inline double
angle_cos(coordinate_t dir0, coordinate_t dir1);

inline double
line_distance(coordinate_t __source, coordinate_t __destination, coordinate_t __point);


/**
 * generates the point s + (d - s) * x
 * @param __source
 * @param __destination
 * @param __relative
 * @return
 */
inline coordinate_t
interpolate_linear(coordinate_t __source, coordinate_t __destination, float __relative);


void WGS84toGoogleBing(double lat, double lon, double &x, double &y);


void GoogleBingtoWGS84Mercator (double x, double y, double &lat, double
&lon);

enum class Projection {
    NONE,
    WGS84_TO_GB,
    GB_TO_WGS84
};


void project_coordinate(coordinate_t& src, Projection projection);
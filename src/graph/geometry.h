#pragma once

#include "base_types.h"

#include "cmath"

// Euclidean distance
double
distance_euclidean(coordinate_t c1, coordinate_t c2);

inline double
to_radians(double degrees);

// exact distance on earths surface
distance_t
distance(coordinate_t c1, coordinate_t c2);

/**
 * angle in radians
 * @param source0
 * @param dest0
 * @param source1
 * @param dest1
 * @return
 */
inline double
angle(coordinate_t source0, coordinate_t dest0, coordinate_t source1, coordinate_t dest1);

inline double
inner_angle(coordinate_t source0, coordinate_t dest0, coordinate_t source1, coordinate_t dest1);

inline double
angle(coordinate_t dir0, coordinate_t dir1);
inline double
inner_angle(coordinate_t dir0, coordinate_t dir1);

[[gnu::hot]]
inline double
angle_cos(coordinate_t const& dir0, coordinate_t const& dir1);

inline double
line_distance(coordinate_t source, coordinate_t destination, coordinate_t point);


/**
 * generates the point s + (d - s) * x
 * @param source
 * @param destination
 * @param relative
 * @return
 */
inline coordinate_t
interpolate_linear(coordinate_t source, coordinate_t destination, double relative);


void WGS84toGoogleBing(double lat, double lon, double &x, double &y);

void GoogleBingtoWGS84Mercator (double x, double y, double &lat, double &lon);

enum class Projection {
    NONE,
    WGS84_TO_GB,
    GB_TO_WGS84
};

void project_coordinate(coordinate_t& src, Projection projection);

inline bool is_in_rectangle(coordinate_t point, coordinate_t bottom_left, coordinate_t top_right);

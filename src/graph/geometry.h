#pragma once

#include "base_types.h"

#include <cstddef>


[[gnu::hot]]
void atan2_approximation(std::size_t num_points, const coordinate_t* coordinates, coordinate_t::component_type out);


namespace std {
    [[gnu::hot]]
    [[gnu::always_inline]]
    [[gnu::const]]
    coordinate_t::component_type atan2(coordinate_t const direction);
}


// Euclidean distance
[[gnu::hot]]
[[gnu::const]]
[[gnu::always_inline]]
inline coordinate_t::component_type
distance_euclidean(coordinate_t c1, coordinate_t c2);

template <typename T>
inline T
to_radians(T degrees);

// exact distance on earths surface
inline coordinate_t::component_type
distance(coordinate_t c1, coordinate_t c2);

/**
 * angle in radians
 * @param source0
 * @param dest0
 * @param source1
 * @param dest1
 * @return
 */
inline coordinate_t::component_type
angle(coordinate_t source0, coordinate_t dest0, coordinate_t source1, coordinate_t dest1);

inline coordinate_t::component_type
inner_angle(coordinate_t source0, coordinate_t dest0, coordinate_t source1, coordinate_t dest1);

inline coordinate_t::component_type
angle(coordinate_t dir0, coordinate_t dir1);
inline coordinate_t::component_type
inner_angle(coordinate_t dir0, coordinate_t dir1);

[[gnu::hot]]
[[gnu::const]]
[[gnu::always_inline]]
inline coordinate_t::component_type
angle_cos(coordinate_t const& dir0, coordinate_t const& dir1);

[[gnu::hot]]
[[gnu::const]]
[[gnu::always_inline]]
inline coordinate_t::component_type
angle_cos_sqr(coordinate_t const& dir0, coordinate_t const& dir1);

inline coordinate_t::component_type
line_distance(coordinate_t source, coordinate_t destination, coordinate_t point);


/**
 * generates the point s + (d - s) * x
 * @param source
 * @param destination
 * @param relative
 * @return
 */
[[gnu::hot]]
[[gnu::always_inline]]
[[gnu::const]]
inline coordinate_t
interpolate_linear(coordinate_t source, coordinate_t destination, coordinate_t::component_type  relative);


void WGS84toGoogleBing(double lat, double lon, double &x, double &y);

void GoogleBingtoWGS84Mercator (double x, double y, double &lat, double &lon);

enum class Projection {
    NONE,
    WGS84_TO_GB,
    GB_TO_WGS84
};

void project_coordinate(coordinate_t& src, Projection projection);

inline bool is_in_rectangle(coordinate_t point, coordinate_t bottom_left, coordinate_t top_right);

#pragma once

#include "geometry.h"
#include "base_types.h"

#include <cmath>


coordinate_t::component_type std::atan2(coordinate_t const& direction) {
    [[assume(std::isnormal(direction.x), std::isnormal(direction.y))]]
    return std::atan2(direction.x, direction.y);
}

// Euclidean distance
coordinate_t::component_type
distance_euclidean(coordinate_t const& c1, coordinate_t c2) {
    c2 -= c1;
    [[assume((std::isnormal(c2.y), std::isnormal(c2.x)))]]
    return std::sqrt(c2.y * c2.y + c2.x * c2.x);
}

inline double
to_radians(double degree) {
    constexpr double one_deg = (std::numbers::pi) / 180;
    return one_deg * degree;
}

// exact distance on earths surface
inline distance_t
distance (coordinate_t const& c1, coordinate_t const& c2) {
    // use euclidean distance, produces better results than haversine (seems to be very unstable)
    return distance_euclidean(c1, c2);
};


/**
 * angle from dir0 to dir1 (counter-clockwise)
 * @param dir0
 * @param dir1
 * @return
 */
inline double
angle(coordinate_t dir0, coordinate_t dir1) {
    // using dot product
    // auto const AB = dir0 * dir1;
    // return std::acos(AB / (dir0.length() * dir1.length()));

    // using atan on both vectors
    auto const angle0 = std::atan2(dir0);
    auto const angle1 = std::atan2(dir1);
    return angle1 - angle0;
}

inline double
inner_angle(coordinate_t dir0, coordinate_t dir1) {
    // using dot product (not working at the moment)
    // auto const dot_product = dir0 * dir1;
    // auto const lengths = std::sqrt(dir0.sqr_length() * dir1.sqr_length());
    // auto const frac = std::clamp(dot_product / lengths, 0.0, 1.0);
    // auto diff = std::acos(frac);

    // using atan on both vectors
    const auto angle0 = std::atan2(dir0);
    const auto angle1 = std::atan2(dir1);
    auto diff = std::fabs(angle1 - angle0);

    diff = (diff > std::numbers::pi) ? (2 * std::numbers::pi) - diff : diff;

    assert(diff == 0 || std::isnormal(diff));
    assert(!std::signbit(diff));
    assert(diff <= std::numbers::pi);
    return diff;
}


inline double
angle_cos(coordinate_t const& dir0, coordinate_t const& dir1) {
    // use dot product
    auto dot_product = (dir0 * dir1);
    return std::sqrt((dot_product * dot_product) / (dir0.sqr_length() * dir1.sqr_length()));
    // return dot_product / (dir0.length() * dir1.length());
}

inline double
angle_cos_sqr(coordinate_t const& dir0, coordinate_t const& dir1) {
    // use dot product
    auto dot_product = (dir0 * dir1);
    return (dot_product * dot_product) / (dir0.sqr_length() * dir1.sqr_length());
}

/**
 * angle from (dest0 - source0) to (dest1 - source1) (counter-clockwise)
 * @param source0
 * @param dest0
 * @param source1
 * @param dest1
 * @return
 */
inline double
angle(const coordinate_t source0, const coordinate_t dest0, const coordinate_t source1, const coordinate_t dest1) {
    // use dot product
    auto const A = dest0 - source0;
    auto const B = dest1 - source1;
    return angle(A, B);
}

inline double
inner_angle(const coordinate_t source0, coordinate_t dest0, const coordinate_t source1, coordinate_t dest1) {
    // use dot product
    dest0 -= source0;
    dest1 -= source1;
    return inner_angle(dest0, dest1);
}


inline double
line_distance(coordinate_t source, coordinate_t destination, coordinate_t point) {
    double phi = inner_angle(source, destination, source, point);

    // approximated distance
    return std::sin(phi) * (point - source).length();
}


/**
 * generates the point s + (d - s) * x
 * @param source
 * @param destination
 * @param relative
 * @return
 */
inline coordinate_t
interpolate_linear(coordinate_t source, coordinate_t destination, double const relative) {
    // return { std::lerp(source.latitude, destination.latitude, relative), std::lerp(source.longitude, destination.longitude, relative) };
    [[assume(std::isnormal(relative))]]
    destination -= source;
    destination *= relative;
    return source + destination;
}


void WGS84toGoogleBing(double lat, double lon, double &x, double &y) {
    x = lon * 20037508.34 / 180;
    y = std::log(std::tan((90 + lat) * std::numbers::pi / 360)) / (std::numbers::pi / 180);
    y = y * 20037508.34 / 180;
}


void GoogleBingtoWGS84Mercator(double x, double y, double &lat, double &lon) {
    lon = (x / 20037508.34) * 180;
    lat = (y / 20037508.34) * 180;

    lat = 180 / std::numbers::pi * (2 * std::atan(std::exp(lat * std::numbers::pi / 180)) - std::numbers::pi / 2);
}

void project_coordinate(coordinate_t &src, Projection projection) {
    switch (projection) {
        case Projection::NONE:
            return;
        case Projection::WGS84_TO_GB:
            WGS84toGoogleBing(src.y, src.x, src.y, src.x);
            return;
        case Projection::GB_TO_WGS84:
            GoogleBingtoWGS84Mercator(src.y, src.x, src.y, src.x);
            return;
    }
}


bool is_in_rectangle(coordinate_t point, coordinate_t bottom_left, coordinate_t top_right) {
    return bottom_left.y <= point.y && point.y <= top_right.y
           && bottom_left.x <= point.x && point.x <= top_right.x;
}

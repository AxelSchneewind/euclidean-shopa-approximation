#pragma once

#include "geometry.h"
#include "base_types.h"

// Euclidean distance
double
distance_euclidean(coordinate_t c1, coordinate_t c2) {
    coordinate_t delta{c2.latitude - c1.latitude, c2.longitude - c1.longitude};
    return std::sqrt(std::pow(delta.latitude, 2) + std::pow(delta.longitude, 2));
}

inline double
to_radians(double __degree) {
    const double one_deg = (M_PI) / 180;
    return one_deg * __degree;
}

// exact distance on earths surface
inline distance_t
distance(coordinate_t __c1, coordinate_t __c2) {
    // for approximated distance (Euclidean distance), produces better results than haversine (seems to be very unstable)
    return distance_euclidean(__c1, __c2);

    // // for exact distances
    // auto dlat = to_radians(__c2.latitude - __c1.latitude);
    // auto dlong = to_radians(__c2.longitude - __c1.longitude);

    // // haversine formula
    // auto ans = std::pow(std::sin(dlat / 2), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlong / 2), 2);
    // ans = 2 * std::asin(std::sqrt(ans));

    // return ans;
};


inline double
angle(coordinate_t dir0, coordinate_t dir1) {
    // using dot product
    // auto const AB = dir0 * dir1;
    // return std::acos(AB / (dir0.length() * dir1.length()));

    // using atan on both vectors
    auto angle0 = std::atan2(dir0.longitude, dir0.latitude);
    auto angle1 = std::atan2(dir1.longitude, dir1.latitude);
    return angle1 - angle0;
}

inline double
inner_angle(coordinate_t dir0, coordinate_t dir1) {
    // using dot product
    // auto const AB = dir0 * dir1;
    // return std::acos(AB / (dir0.length() * dir1.length()));

    // using atan on both vectors
    auto angle0 = std::atan2(dir0.longitude, dir0.latitude);
    auto angle1 = std::atan2(dir1.longitude, dir1.latitude);
    auto diff = std::fabs(angle1 - angle0);

    if (diff > M_PI)
        diff = 2 * M_PI - diff;

    assert(diff >= 0);
    assert(diff <= M_PI);
    return diff;
}


inline double
angle_cos(coordinate_t dir0, coordinate_t dir1) {
    // use dot product
    auto const AB = dir0 * dir1;
    return AB / (dir0.length() * dir1.length());
}

/**
 * angle in radians
 * @param source0
 * @param dest0
 * @param source1
 * @param dest1
 * @return
 */
inline double
angle(coordinate_t source0, coordinate_t dest0, coordinate_t source1, coordinate_t dest1) {
    // use dot product
    auto const A = dest0 - source0;
    auto const B = dest1 - source1;
    return angle(A, B);
}

inline double
inner_angle(coordinate_t source0, coordinate_t dest0, coordinate_t source1, coordinate_t dest1) {
    // use dot product
    dest0 -= source0;
    dest1 -= source1;
    return inner_angle(dest0, dest1);
}


inline double
line_distance(coordinate_t __source, coordinate_t __destination, coordinate_t __point) {
    double phi = inner_angle(__source, __destination, __source, __point);

    // approximated distance
    return std::sin(phi) * (__point - __source).length();
}


/**
 * generates the point s + (d - s) * x
 * @param source
 * @param destination
 * @param relative
 * @return
 */
inline coordinate_t
interpolate_linear(coordinate_t source, coordinate_t destination, float relative) {
    destination -= source;
    destination *= relative;
    return source + destination;
}


void WGS84toGoogleBing(double lat, double lon, double &x, double &y) {
    x = lon * 20037508.34 / 180;
    y = std::log(std::tan((90 + lat) * M_PI / 360)) / (M_PI / 180);
    y = y * 20037508.34 / 180;

}


void GoogleBingtoWGS84Mercator(double x, double y, double &lat, double &lon) {
    lon = (x / 20037508.34) * 180;
    lat = (y / 20037508.34) * 180;

    lat = 180 / M_PI * (2 * std::atan(std::exp(lat * M_PI / 180)) - M_PI / 2);
}

void project_coordinate(coordinate_t &src, Projection projection) {
    switch (projection) {
        case Projection::NONE:
            return;
        case Projection::WGS84_TO_GB:
            WGS84toGoogleBing(src.latitude, src.longitude, src.latitude, src.longitude);
            return;
        case Projection::GB_TO_WGS84:
            GoogleBingtoWGS84Mercator(src.latitude, src.longitude, src.latitude, src.longitude);
            return;
    }
}


bool is_in_rectangle(coordinate_t point, coordinate_t bottom_left, coordinate_t top_right) {
    return bottom_left.latitude <= point.latitude && point.latitude <= top_right.latitude
           && bottom_left.longitude <= point.longitude && point.longitude <= top_right.longitude;
}

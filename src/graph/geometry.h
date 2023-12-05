#pragma once

#include "base_types.h"

#include "cmath"

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

/**
 * angle in radians
 * @param __s0
 * @param __d0
 * @param __s1
 * @param __d1
 * @return
 */
inline double
angle(coordinate_t __s0, coordinate_t __d0, coordinate_t __s1, coordinate_t __d1) {
    // use dot product
    auto const A = __d0 - __s0;
    auto const B = __d1 - __s1;
    auto const AB = A * B;

    return std::acos(AB / (A.length() * B.length()));
}


inline double
line_distance(coordinate_t __source, coordinate_t __destination, coordinate_t __point) {
    double phi = angle(__source, __destination, __source, __point);

    // approximated distance
    return std::sin(phi) * (__point - __source).length();
    // // for exact distance
    // return to_radians(std::sin(phi) * (__point - __source).length()) * 6371;
}


/**
 * generates the point s + (d - s) * x
 * @param __source
 * @param __destination
 * @param __relative
 * @return
 */
inline coordinate_t
interpolate_linear(coordinate_t __source, coordinate_t __destination, float __relative) {
    coordinate_t const delta = __destination - __source;
    return __source + delta * __relative;
}

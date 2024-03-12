#pragma once

#include "geometry.h"
#include "base_types.h"

#include <cmath>


template <typename N>
inline constexpr N atan_scalar_approximation(N x) {
    constexpr N a1  =  0.99997726;
    constexpr N a3  = -0.33262347;
    constexpr N a5  =  0.19354346;
    constexpr N a7  = -0.11643287;
    constexpr N a9  =  0.05265332;
    constexpr N a11 = -0.01172120;

    N x_sq = x*x;
    return x * (a1 + x_sq * (a3 + x_sq * (a5 + x_sq * (a7 + x_sq * (a9 + x_sq * a11)))));
}

void atan2_approximation(size_t num_points, const coordinate_t* coordinates, coordinate_t::component_type* out) {
    for (size_t i = 0; i < num_points; ++i) {
        // Ensure input is in [-1, +1]
        auto y = coordinates[i].latitude;
        auto x = coordinates[i].longitude;
        bool swap = std::fabs(x) < std::fabs(y);
        double atan_input = (swap ? x : y) / (swap ? y : x);

        // Approximate atan
        double res = atan_scalar_approximation(atan_input);

        // If swapped, adjust atan output
        res = swap ? (!std::signbit(atan_input) ? (std::numbers::pi / 2) : -(std::numbers::pi / 2)) - res : res;
        // Adjust quadrants
        //if (x >= 0.0 && y >= 0.0) {}                     // 1st quadrant
        //else if (x < 0.0 && y >= 0.0) { res = M_PI + res; } // 2nd quadrant
        //else if (x < 0.0 && y < 0.0) { res = -M_PI + res; } // 3rd quadrant
        //else if (x >= 0.0 && y < 0.0) {}                     // 4th quadrant
        // simplified
        if (std::signbit(x)) {
            res += (!std::signbit(y) ? (std::numbers::pi) : (-std::numbers::pi));
        }

        // Store result
        out[i] = res;
    }
}

coordinate_t::component_type std::atan2(coordinate_t const direction) {
    return std::atan2(direction.longitude, direction.latitude);
}

// Euclidean distance
coordinate_t::component_type
distance_euclidean(coordinate_t const c1, coordinate_t c2) {
    c2 -= c1;
    return std::sqrt(std::pow(c2.latitude, 2) + std::pow(c2.longitude, 2));
}

inline double
to_radians(double degree) {
    constexpr double one_deg = (std::numbers::pi) / 180;
    return one_deg * degree;
}

// exact distance on earths surface
inline distance_t
distance (coordinate_t c1, coordinate_t c2) {
    // for approximated distance (Euclidean distance), produces better results than haversine (seems to be very unstable)
    return distance_euclidean(c1, c2);

    // // for exact distances
    // auto dlat = to_radians(c2.latitude - c1.latitude);
    // auto dlong = to_radians(c2.longitude - c1.longitude);

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
 * angle in radians
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

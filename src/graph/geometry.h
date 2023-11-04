#pragma once

#include "base_types.h"

#include "cmath"

// euclidian distance
distance_t
distance_euclidian (const coordinate_t &c1, const coordinate_t &c2)
{
  coordinate_t delta{ c2.latitude - c1.latitude, c2.longitude - c1.longitude };
  return std::sqrt (delta.latitude * delta.latitude + delta.longitude * delta.longitude);
}

inline double
to_radians (const double &__degree)
{
  const double one_deg = (M_PI) / 180;
  return one_deg * __degree;
}

// exact distance on earths surface
// TODO fix distances at longitude 180
inline distance_t
distance (const coordinate_t &__c1, const coordinate_t &__c2)
{
  auto lat1 = to_radians (__c1.latitude);
  auto long1 = to_radians (__c1.longitude);
  auto lat2 = to_radians (__c2.latitude);
  auto long2 = to_radians (__c2.longitude);

  auto dlong = long2 - long1;
  auto dlat = lat2 - lat1;

  // haversine formula
  auto ans = std::pow (std::sin (dlat / 2), 2) + std::cos (lat1) * std::cos (lat2) * std::pow (std::sin (dlong / 2), 2);
  ans = 2 * std::asin (std::sqrt (ans));

  // multiply with earths radius
  const int R = 6371;
  ans = ans * R;

  return ans;
};

inline float
angle (const coordinate_t &__s0, const coordinate_t &__d0, const coordinate_t &__s1, const coordinate_t &__d1) {
    // use dot product
    auto A = __d0 - __s0;
    auto B = __d1 - __s1;
    auto AB = A * B;

    return std::acos(AB / (A.length() * B.length()));
}


inline float
line_distance(const coordinate_t& __source, const coordinate_t& __destination, const coordinate_t& __point) {
    auto length = distance(__destination, __source);

    // compute normal vector
    coordinate_t normal = __destination - __source;
    normal = {normal.longitude, - normal.latitude};
    normal = normal * (1 / length);

    return std::abs((__point - __source) * normal);
}


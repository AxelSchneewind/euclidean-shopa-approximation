#pragma once

#include "base_types.h"

#include "cmath"

// euclidian distance
[[deprecated]] distance_t
distance_euclidian (const coordinate_t &c1, const coordinate_t &c2)
{
  coordinate_t delta{ c2.latitude - c1.latitude, c2.longitude - c1.longitude };
  return std::sqrt (delta.latitude * delta.latitude + delta.longitude * delta.longitude);
}

inline distance_t
to_radians (const double &__degree)
{
  const distance_t one_deg = (M_PI) / 180;
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

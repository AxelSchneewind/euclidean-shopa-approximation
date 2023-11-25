#pragma once

#include <limits>
#include <vector>
#include <complex>


using node_id_t = int;
using edge_id_t = int;
using node_level_t = int;

/*
 * a pair of coordinates, consisting of latitude, longitude
 */
struct coordinate_t {
    float latitude;
    float longitude;

    float length() const { return std::sqrt(latitude * latitude + longitude * longitude); }

    coordinate_t operator+(const coordinate_t &second) const {
        return {latitude + second.latitude, longitude + second.longitude};
    }

    coordinate_t operator-(const coordinate_t &second) const {
        return {latitude - second.latitude, longitude - second.longitude};
    }

    float operator*(const coordinate_t &second) const {
        return latitude * second.latitude + longitude * second.longitude;
    }

    coordinate_t operator*(const float &second) const {
        return {latitude * second, longitude * second};
    }

    bool operator==(const coordinate_t &second) const {
        return latitude == second.latitude && longitude == second.longitude;
    }
};


using cost_t = float;
using distance_t = float;

template<typename T>
constexpr T none_value;

template<>
constexpr int none_value<int> = -1;

template<>
constexpr short none_value<short> = -1;

template<typename T>
constexpr bool is_none(T val) { return val == none_value<T>; }


template<typename T>
constexpr T infinity = std::numeric_limits<T>::max();

template<typename T>
constexpr bool is_infinity(T val) { return val == infinity<T>; }

template<typename T>
constexpr T max_cost = 10000;

template<>
constexpr distance_t infinity<distance_t> = (std::numeric_limits<distance_t>::max() / 2) - max_cost<distance_t>;

template<>
constexpr float none_value<float> = infinity<float>;


using triangle = std::array<node_id_t, 3>;


// euclidian distance
distance_t distance(coordinate_t c1, coordinate_t c2);

struct node_t {
    coordinate_t coordinates;

    bool operator==(const node_t &) const = default;
};

struct edge_t {
    cost_t cost;

    edge_t
    invert(const std::vector<edge_id_t> & /*unused*/) const;

    bool operator==(const edge_t &) const = default;
};

struct ch_node_t : public node_t {
    short level;

    bool operator==(const ch_node_t &) const = default;
};

struct ch_edge_t : edge_t {
    edge_id_t edgeA;
    edge_id_t edgeB;

    ch_edge_t
    invert(const std::vector<edge_id_t> &__new_index) const;
};



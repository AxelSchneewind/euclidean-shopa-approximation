#pragma once

#include <cmath>
#include <limits>
#include <vector>
#include <complex>
#include <array>


using node_id_t = int;
using edge_id_t = int;
using node_level_t = int;

/*
 * a pair of coordinates, consisting of latitude, longitude
 */
// TODO use Dim parameter
template <typename C, std::size_t Dim>
struct coordinate {
    using component_type = C;
    static constexpr std::size_t size  = Dim;

    //
    // std::array<component_type, Dim> components;

    component_type latitude;
    component_type longitude;

    [[gnu::always_inline]]
    inline component_type length() const { return std::sqrt((latitude * latitude) + (longitude * longitude)); }
    [[gnu::always_inline]]
    inline component_type sqr_length() const { return (latitude * latitude) + (longitude * longitude); }

    [[gnu::always_inline]]
    inline bool zero() const { return latitude == 0 && longitude == 0; }

    inline coordinate operator+(const coordinate &second) const {
        return { latitude + second.latitude, longitude + second.longitude};
    }
    inline coordinate& operator+=(const coordinate &second) {
        latitude += second.latitude;
        longitude += second.longitude;
        return *this;
    }

    [[gnu::always_inline]]
    inline coordinate operator-(const coordinate &second) const {
        return { latitude - second.latitude, longitude - second.longitude };
    }

    [[gnu::always_inline]]
    inline coordinate& operator-=(const coordinate &second) {
        latitude -= second.latitude;
        longitude -= second.longitude;
        return *this;
    }

    inline double operator*(const coordinate &second) const {
        return latitude * second.latitude + longitude * second.longitude;
    }

    inline coordinate operator*(const component_type &second) const {
        return {latitude * second, longitude * second  };
    }
    inline coordinate& operator*=(const component_type &second) {
        latitude *= second;
        longitude *= second;
        return *this;
    }

    inline bool operator==(const coordinate &second) const {
        return latitude == second.latitude && longitude == second.longitude;
    }
};


using cost_t = double;
using distance_t = double;

template<typename T>
constexpr std::remove_cvref_t<T> none_value;

template<>
constexpr long none_value<long> = -1;

template<>
constexpr int none_value<int> = -1;

template<>
constexpr short none_value<short> = -1;

template<typename T>
constexpr bool is_none(T val) { return val == none_value<T>; }


template<typename T>
constexpr std::remove_cvref_t<T> infinity = std::numeric_limits<std::remove_cvref_t<T>>::max();

template<typename T>
constexpr bool is_infinity(T val) { return val == infinity<T>; }

template<typename T>
constexpr T max_cost = 10000;

template<>
constexpr distance_t infinity<distance_t> = std::numeric_limits<distance_t>::infinity();

template<>
constexpr float none_value<float> = infinity<float>;
template<>
constexpr double none_value<double> = infinity<double>;


using triangle = std::array<node_id_t, 3>;

using coordinate_t = coordinate<double, 2>;

struct node_t {
    coordinate_t coordinates;

    explicit node_t(coordinate_t const& c) : coordinates{c} {}
    explicit node_t(coordinate_t && c) : coordinates{std::move(c)} {}

    node_t() = default;
    node_t(node_t&&) = default;
    node_t(node_t const&) = default;
    node_t& operator=(node_t&&) = default;
    node_t& operator=(node_t const&) = default;

    bool operator==(const node_t &) const = default;
};

struct edge_t {
    cost_t cost;

    explicit edge_t(cost_t const& c) : cost{c} {}

    edge_t() = default;
    edge_t(edge_t&&) = default;
    edge_t(edge_t const&) = default;

    edge_t& operator=(edge_t&&) = default;
    edge_t& operator=(edge_t const&) = default;

    bool operator==(const edge_t &) const = default;
};


struct gl_edge_t {
    int line_width;
    int color;

    bool operator==(const gl_edge_t &) const = default;
};


struct ch_node_t : public node_t {
    short level;

    bool operator==(const ch_node_t &) const = default;
};

struct ch_edge_t : edge_t {
    edge_id_t edgeA;
    edge_id_t edgeB;
};



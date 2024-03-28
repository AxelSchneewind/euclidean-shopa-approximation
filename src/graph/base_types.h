#pragma once

#include "../util/optional.h"

#include <cmath>
#include <limits>
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

    component_type y;
    component_type x;

    inline coordinate& rotate_right() {
        std::swap(y, x);
        x *= -1;
        return *this;
    }

    [[gnu::always_inline]]
    inline component_type length() const { return std::sqrt((y * y) + (x * x)); }
    [[gnu::always_inline]]
    inline component_type sqr_length() const { return (y * y) + (x * x); }

    [[gnu::always_inline]]
    inline bool zero() const { return y == 0 && x == 0; }

    inline coordinate operator+(const coordinate &second) const {
        return {y + second.y, x + second.x};
    }
    inline coordinate& operator+=(const coordinate &second) {
        y += second.y;
        x += second.x;
        return *this;
    }

    [[gnu::always_inline]]
    inline coordinate operator-(const coordinate &second) const {
        return {y - second.y, x - second.x };
    }

    [[gnu::always_inline]]
    inline coordinate& operator-=(const coordinate &second) {
        y -= second.y;
        x -= second.x;
        return *this;
    }

    inline double operator*(const coordinate &second) const {
        return y * second.y + x * second.x;
    }

    inline coordinate operator*(const component_type &second) const {
        return {y * second, x * second  };
    }
    inline coordinate& operator*=(const component_type &second) {
        y *= second;
        x *= second;
        return *this;
    }

    inline bool operator==(const coordinate &second) const {
        return y == second.y && x == second.x;
    }
};


using cost_t = double;
using distance_t = double;

template<>
constexpr long optional::none_value<long> = -1;

template<>
constexpr int optional::none_value<int> = -1;

template<>
constexpr short optional::none_value<short> = -1;


template<typename T>
constexpr std::remove_cvref_t<T> infinity = std::numeric_limits<std::remove_cvref_t<T>>::max();

template<typename T>
constexpr bool is_infinity(T val) { return val == infinity<T>; }

template<typename T>
constexpr T max_cost = 10000;

template<>
constexpr distance_t infinity<distance_t> = std::numeric_limits<distance_t>::infinity();

template<>
constexpr float optional::none_value<float> = infinity<float>;
template<>
constexpr double optional::none_value<double> = infinity<double>;


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



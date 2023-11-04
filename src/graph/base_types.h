#pragma once

#include <limits>
#include <vector>
#include <complex>


/**
 * creates a distinct type from the given base type
 * @tparam Base
 * @tparam Id
 */
template<typename Base, std::size_t Id>
struct distinct_type {
public:
    using base_type = Base;

protected:
    Base _M_value;
public:
    distinct_type() : _M_value() {}

    distinct_type(const Base &val) : _M_value(val) {}

    distinct_type(Base &&val) : _M_value(std::move(val)) {}

    operator const Base &() const { return _M_value; }

    operator Base &() { return _M_value; }

    bool operator==(const Base second) const { return _M_value == second; };

    bool operator<=(const Base second) const { return _M_value <= second; };

    bool operator>=(const Base second) const { return _M_value >= second; };

    bool operator<(const Base second) const { return _M_value < second; };

    bool operator>(const Base second) const { return _M_value > second; };

    bool operator!=(const Base second) const { return _M_value != second; };

    Base operator+(const Base second) const { return _M_value + second; };

    Base operator-(const Base second) const { return _M_value - second; };

    Base operator*(const Base second) const { return _M_value * second; };

    Base operator/(const Base second) const { return _M_value / second; };

    //bool operator==(const distinct_type<Base, Id>&) const = default;
    //bool operator<=(const distinct_type<Base, Id>&) const = default;
    //bool operator>=(const distinct_type<Base, Id>&) const = default;
    //bool operator<(const distinct_type<Base, Id>&) const = default;
    //bool operator>(const distinct_type<Base, Id>&) const = default;
    //bool operator!=(const distinct_type<Base, Id>&) const = default;
};

template<typename Base, std::size_t Id>
std::ostream &operator<<(std::ostream &out, const distinct_type<Base, Id> &obj) {
    return out << (Base) obj;
};


template<typename Base, std::size_t Id>
struct std::hash<distinct_type<Base, Id>> {
    std::size_t operator()(const distinct_type<Base, Id> &val) { return std::hash<Base>{}((Base) val); }
};

//using node_id_t = int;
// using edge_id_t = int;

class node_id_t : public distinct_type<int, 0> {
public:
    static constexpr int NO_NODE_ID = -1;

    node_id_t(int val) : distinct_type<int, 0>(val) {};

    node_id_t() : node_id_t(NO_NODE_ID) {};

    operator bool() const { return _M_value != NO_NODE_ID; }

    operator float() const { return (float)_M_value; };

    operator std::size_t() const { return (std::size_t)_M_value; }

    operator const int &() const { return _M_value; }

    operator int &() { return _M_value; }
};

class edge_id_t : public distinct_type<int, 1> {
public:
    static constexpr int NO_EDGE_ID = -1;

    edge_id_t(int val) : distinct_type<int, 1>(val) {};

    edge_id_t() : edge_id_t(NO_EDGE_ID) {};

    constexpr operator bool() const { return _M_value != NO_EDGE_ID; }

    constexpr operator float() const { return static_cast<float>(_M_value); };

    constexpr operator std::size_t() const { return static_cast<std::size_t>(_M_value); }

    operator const int &() const { return _M_value; }

    operator int &() { return _M_value; }
};

static_assert(!std::convertible_to<node_id_t, edge_id_t>);
static_assert(!std::convertible_to<edge_id_t, node_id_t>);
static_assert(std::convertible_to<node_id_t, bool>);
static_assert(std::convertible_to<edge_id_t, bool>);

template<>
struct std::hash<node_id_t> {
    std::size_t operator()(const node_id_t &val) { return std::hash<node_id_t::base_type>{}(val); }
};

template<>
struct std::hash<edge_id_t> {
    std::size_t operator()(const edge_id_t &val) { return std::hash<edge_id_t::base_type>{}(val); }
};


using node_level_t = distinct_type<int, 2>;


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


/**
 * cost for individual nodes
 */
class cost_t : public distinct_type<float, 3> {
public:
    // make sure that 2* DISTANCE_INF + (any cost value) < max(unsigned int)
    static constexpr float COST_MAX = 1000000;

    cost_t(base_type val) : distinct_type<float, 3>(val) {};

    cost_t() : cost_t(COST_MAX) {};

    operator bool() const { return _M_value != COST_MAX; }

    operator const float &() const { return _M_value; }

    operator float &() { return _M_value; }
};


/**
 * length of a path
 */
class distance_t : public distinct_type<float, 4> {
public:
    // make sure that 2* DISTANCE_INF + (any cost value) < max(unsigned int)
    static constexpr float DISTANCE_INF = (std::numeric_limits<base_type>::max() / 2) -
                                          cost_t::COST_MAX;

    distance_t(base_type val) : distinct_type<float, 4>(val) {};

    distance_t() : distance_t(DISTANCE_INF) {};

    operator bool() const { return _M_value != DISTANCE_INF; }

    operator const float &() const { return _M_value; }

    operator float &() { return _M_value; }

    distance_t& operator+=(const distance_t&__other) { _M_value += (float)__other; return *this; };
};


using triangle = std::array<size_t, 3>;

// euclidian distance
distance_t distance(const coordinate_t &c1, const coordinate_t &c2);

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



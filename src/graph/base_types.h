#pragma once

#include <limits>
#include <vector>
#include <complex>

using node_id_t = int;
using node_level_t = int;
const node_id_t NO_NODE_ID = -1;//std::numeric_limits<node_id_t>::max();

using edge_id_t = int;
const edge_id_t NO_EDGE_ID = -1;//std::numeric_limits<edge_id_t>::max();

/*
 * a pair of coordinates, consisting of latitude, longitude
 */
struct coordinate_t {
    float latitude;
    float longitude;
};

using cost_t = float;
const cost_t COST_MAX = 1000000;

using distance_t = float;
const distance_t DISTANCE_INF = (std::numeric_limits<distance_t>::max() / 2) -
                                COST_MAX;// make sure that 2* DISTANCE_INF + (any cost value) < max(unsigned int)


using triangle = std::array<node_id_t, 3>;

// euclidian distance
distance_t distance(const coordinate_t& c1, const coordinate_t& c2);

struct node_t {
    coordinate_t coordinates;
};

struct edge_t {
    cost_t cost;

    [[nodiscard]] edge_t
    invert(const std::vector<edge_id_t> & /*unused*/) const;
};

struct ch_node_t : node_t {
    node_level_t level;
};

struct ch_edge_t : edge_t {
    edge_id_t edgeA;
    edge_id_t edgeB;

    [[nodiscard]] ch_edge_t
    invert(const std::vector<edge_id_t> &__new_index) const;
};



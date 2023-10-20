#pragma  once

#include <limits>
#include <vector>


typedef int node_id_t;
typedef int node_level_t;
const node_id_t NO_NODE_ID = -1;//std::numeric_limits<node_id_t>::max();

typedef int edge_id_t;
const edge_id_t NO_EDGE_ID = -1;//std::numeric_limits<edge_id_t>::max();

struct coordinate_t { float latitude; float longitude; };

typedef int cost_t;
const cost_t COST_MAX = 1000000;

typedef int distance_t;
const distance_t DISTANCE_INF = (std::numeric_limits<distance_t>::max() / 2) - COST_MAX; // make sure that 2* DISTANCE_INF + (any cost value) < max(unsigned int)





struct node_t {
    coordinate_t coordinates;
};
struct edge_t {
    cost_t cost;

    edge_t invert(const std::vector<edge_id_t>& new_index) const { return edge_t{ cost }; };
};

struct ch_node_t : node_t {
    node_level_t level;
};
struct ch_edge_t : edge_t {
    edge_id_t edgeA; edge_id_t edgeB;

    ch_edge_t invert(const std::vector<edge_id_t>& new_index) const { return ch_edge_t{cost, edgeB != NO_EDGE_ID ? new_index[edgeB] : NO_EDGE_ID, edgeA != NO_NODE_ID ? new_index[edgeA] : NO_NODE_ID }; };
};

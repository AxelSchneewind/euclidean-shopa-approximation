#pragma once

#include "steiner_graph.h"
#include "../graph/graph.h"

/**
 * only stores labels for currently active nodes in a graph with steiner points
 * @tparam NodeCostPair
 */
template<typename NodeCostPair>
class frontier_labels {
private:
    using node_id_type = steiner_graph::node_id_type;
    using base_edge_id_type = steiner_graph::triangle_edge_id_type;
    using distance_type = steiner_graph::distance_type;
    using node_cost_pair_type = NodeCostPair;

    struct label {
        steiner_graph::distance_type distance;
    };

    struct base_edge_info {
        short label_count;
        short outgoing_edge_count;
        std::array<label, std::dynamic_extent> labels;
    };

    std::shared_ptr<const steiner_graph> _M_graph;

    std::unordered_map<steiner_graph::triangle_node_id_type, std::shared_ptr<base_edge_info>> _M_edge_info;

public:
    using label_type = label;

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::shared_ptr<base_edge_info>);

    explicit frontier_labels(std::shared_ptr<const steiner_graph> __graph);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node) {
        _M_edge_info.clear();
    };

    label_type get(node_id_type __node) const {
        return _M_edge_info[__node.edge].labels[__node.steiner_index];
    }

    bool active(base_edge_id_type __edge) const {
        return _M_edge_info.contains(__edge);
    };

    void label(node_cost_pair_type __node_cost_pair) {
        int steiner_index = __node_cost_pair.node.steiner_index;
        base_edge_id_type edge = __node_cost_pair.node.edge;

        if (active(edge)) {
            assert (is_none(_M_edge_info[edge]->labels[steiner_index]));
            _M_edge_info[edge]->label_count++;
            _M_edge_info[edge]->labels[steiner_index] = __node_cost_pair.distance;

            // fully labelled, can remove distances
            if (_M_edge_info[edge]->label_count == _M_edge_info[edge]->outgoing_edge_count)
                _M_edge_info.remove(edge);
        } else {
            int edge_count = 0;
            if (steiner_index > 0) edge_count++;
            if (steiner_index < _M_graph->steiner_info(edge).node_count) edge_count++;
            for (auto other : _M_graph->base_polyhedron().edges(edge)) {
                edge_count += _M_graph->steiner_info(other).node_count * _M_graph->steiner_info(edge).node_count;
            }

            std::vector<label_type> labels(_M_graph->steiner_info(edge).node_count);
            _M_edge_info[edge] = std::make_shared<base_edge_info>({0, edge_count, {labels.begin(), labels.end()} });
        }
    };
};
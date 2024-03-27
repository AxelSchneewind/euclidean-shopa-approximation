#pragma once

#include "fast_map.h"
#include <vector>
#include <memory>
#include <queue>



template <typename Graph>
class node_cache {
    using node_id_type = typename Graph::node_id_type;
    using edge_id_type = typename Graph::edge_id_type;
    using intra_edge_id_type = typename Graph::intra_edge_id_type;

    using coordinate_type = typename Graph::coordinate_type;

    Graph const& _graph;
    size_t _capacity{0};

    fast_map<edge_id_type, intra_edge_id_type, coordinate_type> _coords;

    std::vector<bool> _coords_ready;

    std::queue<edge_id_type> _task_queue;
    std::mutex _queue_access;

    void enqueue(edge_id_type edge) {
        _queue_access.lock();

        _task_queue.push_back(edge);

        _queue_access.unlock();
    }

    void work() {
        _queue_access.lock();

        edge_id_type edge = _task_queue.top();
        _task_queue.pop();

        _queue_access.unlock();

        auto count = _graph.steiner_info(edge).node_count;
        if (_coords.size() + count >= _capacity)
            _coords.reset();
        _coords.append(edge, count);

        for (int i = 0; i < count; ++i) {
            _coords.node_info(edge, i) = _graph.node_coordinates(node_id_type{edge, i});
        }

    }

public:

};
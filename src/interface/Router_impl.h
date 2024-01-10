#pragma once

#include <thread>
#include "Router.h"


Router::Router(const Graph&graph) {
    switch (graph.type()) {
        case GraphType::STEINER_GRAPH_DIRECTED:
        case GraphType::STEINER_GRAPH_UNDIRECTED:
            pimpl = std::make_unique<RouterImplementation<steiner_graph, steiner_routing_t>>(
                graph.get<steiner_graph>(), steiner_routing_t(graph.get<steiner_graph>()));
            break;
        case GraphType::STD_GRAPH_DIRECTED:
        case GraphType::STD_GRAPH_UNDIRECTED:
            pimpl = std::make_unique<RouterImplementation<std_graph_t, std_routing_t>>(
                graph.get<std_graph_t>(), std_routing_t(graph.get<std_graph_t>()));
            break;
    }
}

Router::Router(const Graph&graph, RoutingConfiguration const&config)
    : _config(config) {
    switch (graph.type()) {
        case GraphType::STEINER_GRAPH_DIRECTED:
        case GraphType::STEINER_GRAPH_UNDIRECTED:
            if (_config.use_a_star) {
                using node_cost_pair = node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type,
                    a_star_info>;
                using queue_t = a_star_queue<steiner_graph, node_cost_pair>;
                using labels_t = steiner_labels<steiner_graph, label_type<steiner_graph>>;
                using dijkstra = dijkstra<steiner_graph, queue_t, labels_t, steiner_neighbors<steiner_graph, labels_t>, use_all_edges<steiner_graph>>;
                if (!_config.bidirectional && _config.compact_labels) {
                    using steiner_routing_t = router<steiner_graph, dijkstra>;
                    pimpl = std::make_unique<RouterImplementation<steiner_graph, steiner_routing_t>>(
                        graph.get<steiner_graph>(), steiner_routing_t(graph.get<steiner_graph>()));
                }
                else {
                    using steiner_routing_t = bidirectional_router<steiner_graph, dijkstra>;
                    pimpl = std::make_unique<RouterImplementation<steiner_graph, steiner_routing_t>>(
                        graph.get<steiner_graph>(), steiner_routing_t(graph.get<steiner_graph>()));
                }
            }
            else {
                using node_cost_pair = node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type, void>;
                using queue_t = dijkstra_queue<steiner_graph, node_cost_pair>;
                using labels_t = steiner_labels<steiner_graph, label_type<steiner_graph>>;
                using dijkstra = dijkstra<steiner_graph, queue_t, labels_t, steiner_neighbors<
                    steiner_graph, labels_t>, use_all_edges<steiner_graph>>;
                if (!_config.bidirectional && !_config.compact_labels) {
                    using steiner_routing_t = bidirectional_router<steiner_graph, dijkstra>;
                    pimpl = std::make_unique<RouterImplementation<steiner_graph, steiner_routing_t>>(
                        graph.get<steiner_graph>(), steiner_routing_t(graph.get<steiner_graph>()));
                }
                else {
                }
            }
            break;
        case GraphType::STD_GRAPH_DIRECTED:
        case GraphType::STD_GRAPH_UNDIRECTED:
            pimpl = std::make_unique<RouterImplementation<std_graph_t, a_star_routing_t>>(
                graph.get<std_graph_t>(), a_star_routing_t(graph.get<std_graph_t>()));
            break;
    }
}

template<typename GraphT, typename RouterT>
void Router::RouterImplementation<GraphT, RouterT>::compute_route(long from, long to) {
    _query_ptr = std::make_shared<QueryImplementation<GraphT>>(_graph, from, to);

    _router.init(_query_ptr->from_internal(), _query_ptr->to_internal());

    // create thread to show progress
    bool volatile done = false;
    std::thread status([&]() -> void {
        while (!done) {
            double vm, res;
            process_mem_usage(vm, res);
            std::cout << "\rdistances: "
                    << std::setw(12) /*<< std::setprecision(3)*/ << _router.forward_distance()
                    << " (" << std::setw(12) /*<< std::setprecision(3)*/
                    << _router.forward_current().value() << "), of "
                    << _query_ptr->beeline_distance() << ", ";

            if constexpr (requires(RouterT::labels_type&&l) { l.aggregate_count(); }) {
                std::cout << "node aggregates currently expanded: " << std::setw(10)
                        << _router.forward_labels().aggregate_count() +
                        _router.backward_labels().aggregate_count();
            }

            std::cout << "memory usage : VM " << std::setw(9) << vm / 1024 << "MiB, RES " << std::setw(9)
                    << res / 1024
                    << "MiB" << std::flush;
            usleep(100000);
        }

        std::cout << ", done" << std::endl;
    });


    auto before = std::chrono::high_resolution_clock::now();

    _router.compute_route();

    auto after = std::chrono::high_resolution_clock::now();
    done = true;
    status.join();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    _result_ptr = std::make_shared<ResultImplementation<GraphT>>(_graph, *_query_ptr, _router, duration);
}

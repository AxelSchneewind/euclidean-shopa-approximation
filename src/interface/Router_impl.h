#pragma once

#include <thread>
#include "Router.h"

Router::Router(const Graph&graph, RoutingConfiguration const&config)
    : _config(config) {
    switch (graph.type()) {
        case GraphType::STEINER_GRAPH_DIRECTED:
        case GraphType::STEINER_GRAPH_UNDIRECTED:
            if (_config.use_a_star) {
                using node_cost_pair = geometric_node_cost_pair<node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type,
                    a_star_info>, steiner_graph>;
                using queue_t = a_star_queue<steiner_graph, node_cost_pair>;
                if (_config.compact_labels) {   // map based labels
                    using labels_t = steiner_labels<steiner_graph, label_type<steiner_graph>>; // TODO
                    using dijkstra = dijkstra<steiner_graph, queue_t, labels_t, steiner_neighbors<steiner_graph, labels_t>>;
                    if (!_config.bidirectional) { // unidirectional routing
                        using steiner_routing_t = router<steiner_graph, dijkstra>;
                        impl = std::make_shared<RouterImplementation < steiner_graph, steiner_routing_t>>( graph.get_implementation<steiner_graph>(), steiner_routing_t(graph.get_implementation<steiner_graph>()), _config);
                    } else { // bidirectional routing
                        using steiner_routing_t = bidirectional_router<steiner_graph, dijkstra>;
                        impl = std::make_shared<RouterImplementation < steiner_graph, steiner_routing_t>> ( graph.get_implementation<steiner_graph>(), steiner_routing_t(graph.get_implementation<steiner_graph>()), _config);
                    }
                } else { // array-based labels
                    using labels_t = steiner_labels<steiner_graph, label_type<steiner_graph>>; // TODO
                    using dijkstra = dijkstra<steiner_graph, queue_t, labels_t, steiner_neighbors<steiner_graph, labels_t>>;
                    using steiner_routing_t = router<steiner_graph, dijkstra>;
                    impl = std::make_shared<RouterImplementation<steiner_graph, steiner_routing_t>>(
                        graph.get_implementation<steiner_graph>(), steiner_routing_t(graph.get_implementation<steiner_graph>()), _config);
                }
            } else {
                using node_cost_pair = geometric_node_cost_pair<node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type, void>, steiner_graph>;
                using queue_t = dijkstra_queue<steiner_graph, node_cost_pair>;
                if (_config.compact_labels) {   // map based labels
                    using labels_t = steiner_labels<steiner_graph, label_type<steiner_graph>>; // TODO
                    using dijkstra = dijkstra<steiner_graph, queue_t, labels_t, steiner_neighbors<steiner_graph, labels_t>>;
                    if (!_config.bidirectional) { // unidirectional routing
                        using steiner_routing_t = router<steiner_graph, dijkstra>;
                        impl = std::make_shared<RouterImplementation < steiner_graph, steiner_routing_t>>( graph.get_implementation<steiner_graph>(), steiner_routing_t(graph.get_implementation<steiner_graph>()), _config);
                    } else { // bidirectional routing
                        using steiner_routing_t = bidirectional_router<steiner_graph, dijkstra>;
                        impl = std::make_shared<RouterImplementation < steiner_graph, steiner_routing_t>> ( graph.get_implementation<steiner_graph>(), steiner_routing_t(graph.get_implementation<steiner_graph>()), _config);
                    }
                } else { // array-based labels
                    using labels_t = steiner_labels<steiner_graph, label_type<steiner_graph>>; // TODO
                    using dijkstra = dijkstra<steiner_graph, queue_t, labels_t, steiner_neighbors<steiner_graph, labels_t>>;
                    using steiner_routing_t = router<steiner_graph, dijkstra>;
                    impl = std::make_shared<RouterImplementation<steiner_graph, steiner_routing_t>>(
                        graph.get_implementation<steiner_graph>(), steiner_routing_t(graph.get_implementation<steiner_graph>()), _config);
                }
            }
            break;
        case GraphType::STD_GRAPH_DIRECTED:
        case GraphType::STD_GRAPH_UNDIRECTED:
            if (_config.use_a_star) {
                impl = std::make_shared<RouterImplementation < std_graph_t, a_star_routing_t>> ( graph.get_implementation<std_graph_t>(), a_star_routing_t(graph.get_implementation<std_graph_t>()), _config);
            } else { // don't use A*
                impl = std::make_shared<RouterImplementation < std_graph_t, std_routing_t>> ( graph.get_implementation<std_graph_t>(), std_routing_t(graph.get_implementation<std_graph_t>()), _config);
            }
            break;
    }

    if (!impl) {
        throw std::invalid_argument("No implementation available for the given config");
    }
}

template<typename GraphT, typename RouterT>
void Router::RouterImplementation<GraphT, RouterT>::compute_route(long from, long to) {
    QueryImplementation<GraphT> query_impl(_graph, from, to, _config);
    Query query(std::make_shared<QueryImplementation<GraphT>>(query_impl));
    perform_query(query);
}

template<typename GraphT, typename RouterT>
void Router::RouterImplementation<GraphT, RouterT>::perform_query(const Query &query) {
    _query_ptr = std::make_shared<QueryImplementation<GraphT>>(query.get_implementation<QueryImplementation<GraphT>>());
    _router.init(_query_ptr->from_internal(), _query_ptr->to_internal());

    // create thread to show progress
    std::thread status;
    bool volatile done = false;
    if (_config.live_status) {
        status = std::thread([&] () -> void {
            while (!done) {
                double vm, res;
                process_mem_usage(vm, res);
                std::cout << "\rdistances: "
                          << std::setw(12) /*<< std::setprecision(3)*/ << _router.forward_distance()
                          << " (" << std::setw(12) /*<< std::setprecision(3)*/
                          << _router.forward_current().value() << "), of "
                          << _query_ptr->beeline_distance() << ", ";

                if constexpr (requires(RouterT::labels_type && l) { l.aggregate_count(); }) {
                    std::cout << "node aggregates currently expanded: " << std::setw(10)
                              << _router.forward_labels().aggregate_count() +
                                 _router.backward_labels().aggregate_count();
                }

                std::cout << "memory usage : VM " << std::setw(9) << vm / 1024 << "MiB" << std::flush;
                usleep(100000);
            }

            std::cout << ", done" << std::endl;
        });
    }


    auto const before = std::chrono::high_resolution_clock::now();

    _router.compute();

    auto const after = std::chrono::high_resolution_clock::now();
    done = true;
    if (_config.live_status)
        status.join();

    auto const duration = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    _result_ptr = std::make_shared<ResultImplementation<GraphT>>(_graph, *_query_ptr, _router, duration);
}

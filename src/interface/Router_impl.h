#pragma once

#include "Router.h"

#include "../routing_impl.h"
#include "../routing/dijkstra_concepts.h"

#include <thread>
#include "../routing/node_label.h"


template<typename GraphImplementation, bool only_distance, bool use_a_star, Configuration config>
struct Implementation {
    using graph_t = void;
    using routing_t = void;
    using queue_t = void;
    using labels_t = void;
    using neighbors_t = void;
    using dijkstra_t = void;
};


template<bool only_distance, bool use_a_star, Configuration n>
struct Implementation<steiner_graph, only_distance, use_a_star, n> {
    using graph_t = steiner_graph;
    using node_id_t = typename graph_t::node_id_type;
    using base_node_id_t = typename graph_t::triangle_node_id_type;
    using distance_t = typename graph_t::distance_type;

    struct ncp_impl {
        distance_t _distance;
        distance_t _heuristic;
        node_id_t _node{optional::none_value<node_id_t>};
    };

    struct label_impl {
        distance_t _distance{infinity<distance_t>};
        distance_t _heuristic{infinity<distance_t>};
        node_id_t _predecessor{optional::none_value<node_id_t>};
        node_id_t _face_crossing_predecessor{optional::none_value<node_id_t>};
    };

    using node_cost_pair_t = node_label<ncp_impl>;
    using label_t = node_label<label_impl>;

    // check that no space is wasted due to node_label<>
    static_assert(sizeof(label_impl) == sizeof(label_t));
    static_assert(alignof(label_impl) == alignof(label_t));
    static_assert(sizeof(ncp_impl) == sizeof(node_cost_pair_t));
    static_assert(alignof(ncp_impl) == alignof(node_cost_pair_t));

    // using node_cost_pair_t = geometric_node_cost_pair<node_id_t, distance_t, float, node_id_t>;
    using labels_t = steiner_labels<steiner_graph, label_t>;
    using queue_t = dijkstra_queue<node_cost_pair_t, compare_heuristic>;
    using neighbors_t = steiner_neighbors<steiner_graph, labels_t, n>;
    using dijkstra_t = dijkstra<steiner_graph, queue_t, labels_t, neighbors_t, a_star_heuristic<steiner_graph>>;
    using routing_t = router<steiner_graph, dijkstra_t>;
};

template<bool use_a_star, bool bidirectional, Configuration n>
struct Implementation<steiner_graph, true, use_a_star, bidirectional, n> {
    using graph_t = steiner_graph;
    using node_id_t = typename graph_t::node_id_type;
    using base_node_id_t = typename graph_t::triangle_node_id_type;
    using distance_t = typename graph_t::distance_type;

    struct ncp_impl {
        distance_t _distance;
        distance_t _heuristic;
        node_id_t _node{optional::none_value<node_id_t>};
    };

    struct label_impl {
        distance_t _distance{infinity<distance_t>};
        // distance_t _heuristic{infinity<distance_t>};
        // node_id_t _predecessor{optional::none_value<node_id_t>};
        node_id_t _face_crossing_predecessor{optional::none_value<node_id_t>};
    };

    using node_cost_pair_t = node_label<ncp_impl>;
    using label_t = node_label<label_impl>;

    // check that no space is wasted due to node_label<>
    static_assert(sizeof(label_impl) == sizeof(label_t));
    static_assert(alignof(label_impl) == alignof(label_t));
    static_assert(sizeof(ncp_impl) == sizeof(node_cost_pair_t));
    static_assert(alignof(ncp_impl) == alignof(node_cost_pair_t));

    // using node_cost_pair_t = geometric_node_cost_pair<node_id_t, distance_t, float, node_id_t>;
    using labels_t = frontier_labels<node_cost_pair_t, label_t>;
    using queue_t = dijkstra_queue<node_cost_pair_t, compare_heuristic>;
    using neighbors_t = steiner_neighbors<steiner_graph, labels_t, n>;
    using dijkstra_t = dijkstra<steiner_graph, queue_t, labels_t, neighbors_t, a_star_heuristic<steiner_graph>>;
    using routing_t = router<steiner_graph, dijkstra_t>;
};


template<bool only_distances, Configuration n>
struct Implementation<steiner_graph, only_distances, false, n> {
    using graph_t = steiner_graph;
    using node_id_t = typename graph_t::node_id_type;
    using base_node_id_t = typename graph_t::triangle_node_id_type;
    using distance_t = typename graph_t::distance_type;


    struct ncp_impl {
        distance_t _distance;
        node_id_t _node{optional::none_value<node_id_t>};
    };

    struct label_impl {
        distance_t _distance{infinity<distance_t>};
        node_id_t _predecessor{optional::none_value<node_id_t>};
        node_id_t _face_crossing_predecessor{optional::none_value<node_id_t>};
    };

    using node_cost_pair_t = node_label<ncp_impl>;
    using label_t = node_label<label_impl>;

    using queue_t = dijkstra_queue<node_cost_pair_t, compare_distance>;
    using labels_t = steiner_labels<graph_t, label_t>;
    using neighbors_t = steiner_neighbors<graph_t, labels_t, n>;
    using dijkstra_t = dijkstra<graph_t, queue_t, labels_t, neighbors_t>;
    using routing_t = router<graph_t, dijkstra_t>;
};


template<bool only_distance, bool use_a_star, Configuration n>
struct Implementation<std_graph_t, only_distance, use_a_star, n> {
    using graph_t = std_graph_t;
    using node_id_t = typename graph_t::node_id_type;
    using distance_t = typename graph_t::distance_type;

    struct ncp_impl {
        node_id_t _node{optional::none_value<node_id_t>};
        node_id_t _predecessor{optional::none_value<node_id_t>};
        distance_t _distance;
        distance_t _heuristic;
    };

    struct label_impl {
        distance_t _distance{infinity<distance_t>};
        node_id_t _predecessor{optional::none_value<node_id_t>};
    };

    using node_cost_pair_t = node_label<ncp_impl>;
    using label_t = node_label<label_impl>;


    using queue_t = dijkstra_queue<node_cost_pair_t, compare_heuristic>;
    using labels_t = node_labels<std_graph_t, label_t>;
    using neighbors_t = default_neighbors<graph_t>;
    using dijkstra_t = dijkstra<graph_t, queue_t, labels_t, neighbors_t, a_star_heuristic<std_graph_t>>;
    using routing_t = router<graph_t, dijkstra_t>;
};

template<typename GraphImpl, bool use_a_star, bool only_distance, Configuration n>
std::unique_ptr<RouterInterface> make_router(Graph const &graph) {
    using steiner_routing_t = typename Implementation<GraphImpl, only_distance, use_a_star, n>::routing_t;
    return std::make_unique<RouterImplementation < steiner_graph, steiner_routing_t> >
           (graph.get_implementation<steiner_graph>(), steiner_routing_t(
                   graph.get_implementation<steiner_graph>()), _config);
}

// TODO clean up this mess
Router::Router(const Graph &graph, RoutingConfiguration const &config)
        : _config(config) {
    switch (graph.type()) {
        case GraphType::STEINER_GRAPH:
            if (_config.only_distance) {
                if (_config.use_a_star) {
                    switch (_config.min_angle_neighbor_method) {
                        case RoutingConfiguration::PARAM: {
                            impl = make_router<steiner_graph, true, true, Configuration::PARAM>(graph);
                            break;
                        }
                        case RoutingConfiguration::BINSEARCH: {
                            impl = make_router<steiner_graph, true, true, Configuration::BINSEARCH>(graph);
                            break;
                        }
                        case RoutingConfiguration::ATAN2: {
                            impl = make_router<steiner_graph, true, true, Configuration::ATAN2>(graph);
                            break;
                        }
                        case RoutingConfiguration::LINEAR: {
                            impl = make_router<steiner_graph, true, true, Configuration::LINEAR>(graph);
                            break;
                        }
                    }
                } else { // A* disabled
                    impl = make_router<steiner_graph, false, true, Configuration::PARAM>(graph);
                }
            } else { // build tree
                if (_config.use_a_star) {
                    switch (_config.min_angle_neighbor_method) {
                        case RoutingConfiguration::PARAM: {
                            impl = make_router<steiner_graph, true, false, Configuration::PARAM>(graph);
                            break;
                        }
                        case RoutingConfiguration::BINSEARCH: {
                            impl = make_router<steiner_graph, true, false, Configuration::BINSEARCH>(graph);
                            break;
                        }
                        case RoutingConfiguration::ATAN2: {
                            impl = make_router<steiner_graph, true, false, Configuration::ATAN2>(graph);
                            break;
                        }
                        case RoutingConfiguration::LINEAR: {
                            impl = make_router<steiner_graph, true, false, Configuration::LINEAR>(graph);
                            break;
                        }
                    }
                } else { // A* disabled
                    impl = make_router<steiner_graph, false, false, Configuration::PARAM>(graph);
                }
            }
            break;
        case GraphType::STD_GRAPH:
            if (_config.use_a_star) {
                impl = make_router<std_graph_t, true, false, Configuration::PARAM>(graph);
            } else { // don't use A*
                impl = make_router<std_graph_t, false, false, Configuration::PARAM>(graph);
            }
            break;
        default:
            throw std::runtime_error("can't create router for this graph type");
    }

    if (!impl) {
        throw std::invalid_argument("No implementation available for the given config");
    }
}

template<typename GraphT, typename RouterT>
void Router::RouterImplementation<GraphT, RouterT>::compute_route(long from, long to) {
    QueryImplementation<GraphT> query_impl(*_graph, from, to, _config);
    Query query(std::make_shared<QueryImplementation<GraphT>>(query_impl));
    perform_query(query);
}

template<typename GraphT, typename RouterT>
void Router::RouterImplementation<GraphT, RouterT>::compute_route(long from, long to, std::ostream &out) {
    QueryImplementation<GraphT> query_impl(*_graph, from, to, _config);
    Query query(std::make_shared<QueryImplementation<GraphT>>(query_impl));
    _query_ptr = std::make_shared<QueryImplementation<GraphT>>(query.get_implementation<QueryImplementation<GraphT>>());

    auto const before = std::chrono::high_resolution_clock::now();

    out << "node," << from << '\n';
    _router.init(_query_ptr->from_internal(), _query_ptr->to_internal());
    while (!_router.done()) {
        if constexpr (requires(GraphT g, typename GraphT::node_id_type n) { g.is_base_node(n); g.base_node_id(n); }) {
            if (_graph->is_base_node(_router.forward_current().node()))
                out << _graph->base_node_id(_router.forward_current().node()) << ','
                    << _router.forward_current().distance() << '\n';
        } else {
            out << _router.forward_current().node() << ',' << _router.forward_current().distance() << '\n';
        }

        _router.step_forward();
    }

    auto const after = std::chrono::high_resolution_clock::now();
    auto const duration = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);

    _result_ptr = std::make_shared<ResultImplementation<GraphT>>(*_graph, *_query_ptr, _router, duration);
}

template<typename GraphT, typename RouterT>
void Router::RouterImplementation<GraphT, RouterT>::perform_query(const Query &query) {
    _query_ptr = std::make_shared<QueryImplementation<GraphT>>(query.get_implementation<QueryImplementation<GraphT>>());
    _router.init(_query_ptr->from_internal(), _query_ptr->to_internal());

    // create thread to show progress
    std::thread status;
    bool volatile done = false;
    if (_config.live_status) {
        status = std::thread([&done, this]() -> void {
            while (!done) {
                double vm, res;
                process_mem_usage(vm, res);
                std::cout << "\rdistances: "
                          << std::setw(12) << _router.forward_distance();

                if constexpr (requires { requires HasHeuristic<typename RouterT::node_cost_pair_type>; }) {
                    std::cout << " (" << std::setw(12)
                              << _router.forward_current().heuristic() << ") ";
                }
                std::cout << " of "
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
    _result_ptr = std::make_shared<ResultImplementation<GraphT>>(*_graph, *_query_ptr, _router, duration);
}

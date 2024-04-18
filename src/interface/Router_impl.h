#pragma once

#include "Router.h"

#include "../routing_impl.h"
#include "../routing/dijkstra_concepts.h"

#include <thread>
#include "../routing/node_label.h"


template<bool condition, typename True, typename False>
struct if_or_else {
    using type = void;
};

template<typename True, typename False>
struct if_or_else<true, True, False> {
    using type = True;
};

template<typename True, typename False>
struct if_or_else<false, True, False> {
    using type = False;
};


template<typename GraphImplementation, bool only_distance, bool use_a_star, NeighborFindingAlgorithm config, Pruning simplifications>
struct Implementation {
    using graph_t = void;
    using routing_t = void;
    using queue_t = void;
    using labels_t = void;
    using neighbors_t = void;
    using dijkstra_t = void;
};


template<bool only_distance, bool use_a_star, NeighborFindingAlgorithm n, Pruning simplifications>
struct Implementation<steiner_graph, only_distance, use_a_star, n, simplifications> {
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
    using queue_t = dijkstra_queue<node_cost_pair_t, typename if_or_else<use_a_star, compare_heuristic, compare_distance>::type>;
    using neighbors_t = steiner_neighbors<steiner_graph, labels_t, simplifications, n>;
    using dijkstra_t = dijkstra<steiner_graph, queue_t, labels_t, neighbors_t, typename if_or_else<use_a_star, a_star_heuristic<steiner_graph>, no_heuristic>::type>;
    using routing_t = router<steiner_graph, dijkstra_t>;
};

template<bool use_a_star, NeighborFindingAlgorithm n, Pruning simplifications>
struct Implementation<steiner_graph, true, use_a_star, n, simplifications> {
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
        node_id_t _face_crossing_predecessor{optional::none_value<node_id_t>};
    };

    using node_cost_pair_t = node_label<ncp_impl>;
    using label_t = node_label<label_impl>;

    using labels_t = frontier_labels<node_cost_pair_t, label_t>;
    using queue_t = dijkstra_queue<node_cost_pair_t, typename if_or_else<use_a_star, compare_heuristic, compare_distance>::type>;
    using neighbors_t = steiner_neighbors<steiner_graph, labels_t, simplifications, n>;
    using dijkstra_t = dijkstra<steiner_graph, queue_t, labels_t, neighbors_t, typename if_or_else<use_a_star, a_star_heuristic<steiner_graph>, no_heuristic>::type>;
    using routing_t = router<steiner_graph, dijkstra_t>;
};


template<bool only_distance, bool use_a_star, NeighborFindingAlgorithm n, Pruning simplifications>
struct Implementation<std_graph_t, only_distance, use_a_star, n, simplifications> {
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


    using queue_t = dijkstra_queue<node_cost_pair_t, typename if_or_else<use_a_star, compare_heuristic, compare_distance>::type>;
    using labels_t = node_labels<std_graph_t, label_t>;
    using neighbors_t = default_neighbors<graph_t>;
    using dijkstra_t = dijkstra<graph_t, queue_t, labels_t, neighbors_t, typename if_or_else<use_a_star, a_star_heuristic<std_graph_t>, no_heuristic>::type>;
    using routing_t = router<graph_t, dijkstra_t>;
};

template<typename GraphImpl, bool use_a_star, bool only_distance, Pruning simplifications, NeighborFindingAlgorithm algorithm>
std::unique_ptr<RouterInterface> make_router(Graph const &graph, RoutingConfiguration const& config) {
    if constexpr(std::same_as<typename Implementation<GraphImpl, only_distance, use_a_star, algorithm, simplifications>::graph_t, void>) {
        return {};
    } else {
        using routing_t = typename Implementation<GraphImpl, only_distance, use_a_star, algorithm, simplifications>::routing_t;
        return std::make_unique<Router::RouterImplementation<GraphImpl, routing_t>>(
                graph.get_implementation<GraphImpl>(), routing_t(graph.get_implementation<GraphImpl>()), config);
    }
}

template<typename GraphImpl, bool use_a_star, bool only_distance, Pruning simplifications>
std::unique_ptr<RouterInterface> by_algorithm(Graph const& graph, RoutingConfiguration const& config) {
    switch (config.neighbor_selection_algorithm) {
        case RoutingConfiguration::NeighborFindingAlgorithm::LINEAR:
            return make_router<GraphImpl, use_a_star, only_distance, simplifications, NeighborFindingAlgorithm::LINEAR>(graph, config);
        case RoutingConfiguration::NeighborFindingAlgorithm::BINSEARCH:
            return make_router<GraphImpl, use_a_star, only_distance, simplifications, NeighborFindingAlgorithm::BINSEARCH>(graph, config);
        case RoutingConfiguration::NeighborFindingAlgorithm::ATAN2:
            return make_router<GraphImpl, use_a_star, only_distance, simplifications, NeighborFindingAlgorithm::ATAN2>(graph, config);
        case RoutingConfiguration::NeighborFindingAlgorithm::PARAM:
            return make_router<GraphImpl, use_a_star, only_distance, simplifications, NeighborFindingAlgorithm::PARAM>(graph, config);
    }
    return {};
}

template<typename GraphImpl, bool use_a_star, bool only_distance>
std::unique_ptr<RouterInterface> by_pruning(Graph const& graph, RoutingConfiguration const& config) {
    switch (config.pruning) {
        case RoutingConfiguration::Pruning::UNPRUNED:
            return by_algorithm<GraphImpl, use_a_star, only_distance, Pruning::UNPRUNED>(graph, config);
        case RoutingConfiguration::Pruning::PRUNE_DEFAULT:
            return by_algorithm<GraphImpl, use_a_star, only_distance, Pruning::PRUNE_DEFAULT>(graph, config);
        case RoutingConfiguration::Pruning::MinBendingAngleESpanner:
            return by_algorithm<GraphImpl, use_a_star, only_distance, Pruning::MinBendingAngleESpanner>(graph, config);
    }
    return {};
}

template<typename GraphImpl, bool use_a_star>
std::unique_ptr<RouterInterface> by_only_distance(Graph const& graph, RoutingConfiguration const& config) {
    if (config.only_distance) {
        return by_pruning<GraphImpl, use_a_star, true>(graph, config);
    } else {
        return by_pruning<GraphImpl, use_a_star, true>(graph, config);
    }
    return {};
}

template<typename GraphImpl>
std::unique_ptr<RouterInterface> by_a_star(Graph const& graph, RoutingConfiguration const& config) {
    if (config.use_a_star) {
        return by_only_distance<GraphImpl, true>(graph, config);
    } else {
        return by_only_distance<GraphImpl, false>(graph, config);
    }
    return {};
}

std::unique_ptr<RouterInterface> by_graph_impl(Graph const& graph, RoutingConfiguration const& config) {
    switch (graph.type()) {
        case GraphType::STEINER_GRAPH:
            return by_a_star<steiner_graph>(graph, config);
        case GraphType::STD_GRAPH:
            return by_a_star<std_graph_t>(graph, config);
        case GraphType::NONE:
            return {};
    }
    return {};
}


std::unique_ptr<RouterInterface> select_routing_impl(Graph const& graph, RoutingConfiguration const& config) {
    return by_graph_impl(graph, config);
}


Router::Router(const Graph &graph, RoutingConfiguration const &config)
        : _config(config) {
    impl = select_routing_impl(graph, config);

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
                              << _router.forward_labels().aggregate_count();
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

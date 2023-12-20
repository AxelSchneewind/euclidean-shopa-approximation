#pragma once

#include "Router.h"


Router::Router(const Graph &graph) {
    switch (graph.type()) {
        case GraphType::STEINER_GRAPH_DIRECTED:
        case GraphType::STEINER_GRAPH_UNDIRECTED:
            pimpl = std::make_unique<RouterImplementation<steiner_graph, steiner_routing_t>>(graph, steiner_routing_t(graph.get<const steiner_graph&>()));
            break;
        case GraphType::STD_GRAPH_DIRECTED:
        case GraphType::STD_GRAPH_UNDIRECTED:
            pimpl = std::make_unique<RouterImplementation<std_graph_t, std_routing_t>>(graph, std_routing_t(graph.get<const std_graph_t&>()));
            break;
    }
}



template<typename GraphT, typename RouterT>
void Router::RouterImplementation<GraphT, RouterT>::compute_route(long from, long to) {
    _query = QueryImplementation<GraphT>(_graph.get<GraphT const&>(), from, to);

    _router.init(_query.from_internal(), _query.to_internal());
    auto before = std::chrono::high_resolution_clock::now();

    _router.compute_route();

    auto after = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    _result = ResultImplementation<GraphT>(_graph.get<GraphT const&>(), _query, _router, duration);
}

#pragma once

#include <thread>
#include "Router.h"


Router::Router(const Graph &graph) {
    switch (graph.type()) {
        case GraphType::STEINER_GRAPH_DIRECTED:
        case GraphType::STEINER_GRAPH_UNDIRECTED:
            pimpl = std::make_unique<RouterImplementation < steiner_graph, steiner_routing_t>> (graph.get<steiner_graph>(), steiner_routing_t(graph.get<steiner_graph>()));
            break;
        case GraphType::STD_GRAPH_DIRECTED:
        case GraphType::STD_GRAPH_UNDIRECTED:
            pimpl = std::make_unique<RouterImplementation < std_graph_t, std_routing_t>> (graph.get<std_graph_t>(), std_routing_t(graph.get<std_graph_t>()));
            break;
    }
}


template<typename GraphT, typename RouterT>
void Router::RouterImplementation<GraphT, RouterT>::compute_route(long from, long to) {
    _query_ptr = std::make_shared<QueryImplementation<GraphT>>(_graph, from, to);

    std::cout << "computing route from node " << from << " to " << to << std::endl;
    _router.init(_query_ptr->from_internal(), _query_ptr->to_internal());

    // create thread to show progress
    bool done = false;
    std::thread status([&]() -> void {
        while (!done) {
            double vm, res;
            process_mem_usage(vm, res);
            std::cout << "\rdistances: "
                      << std::setw(12) /*<< std::setprecision(3)*/ << _router.forward_distance()
                      << " (" << std::setw(12) /*<< std::setprecision(3)*/
                      << _router.forward_current().value() << "), "
                      << std::setw(12) /*<< std::setprecision(3)*/ << _router.backward_distance()
                      << " (" << std::setw(12) /*<< std::setprecision(3)*/
                      << _router.backward_current().value() << ")";

            if constexpr (requires(RouterT::labels_type && l) { l.aggregate_count(); }) {
                std::cout << ", node aggregates currently expanded: " << std::setw(10)
                          << _router.forward_labels().aggregate_count() +
                             _router.backward_labels().aggregate_count();
            }

            std::cout << ", memory usage : VM " << std::setw(9) << vm / 1024 << "MiB, RES " << std::setw(9)
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
    std::cout << "search took " << duration << std::endl;
    _result_ptr = std::make_shared<ResultImplementation<GraphT>>(_graph, *_query_ptr, _router, duration);
}
#pragma once

#include "Graph.h"
#include "Query.h"

#include "../util/memory_usage.h"
#include "../routing_impl.h"


template<typename GraphT>
GraphType Graph::GraphImplementation<GraphT>::type() const {
    return GraphType::STD_GRAPH_DIRECTED;
}

template<>
GraphType Graph::GraphImplementation<steiner_graph>::type() const {
    return GraphType::STEINER_GRAPH_UNDIRECTED;
}




template<RoutableGraph GraphT>
template<typename RouterT>
ResultImplementation<GraphT>::ResultImplementation(const GraphT &graph, QueryImplementation<GraphT> query,
                                                   const RouterT &router,
                                                   std::chrono::duration<double, std::milli> duration)
        : _query(query)
        , _route_found(router.route_found())
        , _path(std_graph_t::make_graph(graph, graph.make_subgraph(router.route())))
        , _tree_forward(std_graph_t::make_graph(graph, router.tree_forward()))
        , _tree_backward(std_graph_t::make_graph(graph, router.tree_backward()))
        , _nodes_visited(_tree_forward.node_count() + _tree_backward.node_count())
        , _distance(router.distance())
        , _duration(duration) {}



template<>
void Graph::GraphImplementation<steiner_graph>::write_graph_stats(std::ostream &output) const {
    output << "\r\agraph has "
           << std::setw(12) << graph.node_count() << " nodes and "
           << std::setw(12) << graph.edge_count() / 2 << " edges"
           << "\n                 with "
           << std::setw(12) << graph.base_graph().node_count() << " nodes and "
           << std::setw(12) << graph.base_graph().edge_count() / 2 << " edges stored explicitly (ε = "
           << graph.epsilon() << ")" << std::endl;

    output << "    graph:  expected size per node: "
           << std::setw(5) << steiner_graph::SIZE_PER_NODE << " and per edge "
           << std::setw(5) << steiner_graph::SIZE_PER_EDGE << " -> "
           << graph.base_graph().node_count() * steiner_graph::SIZE_PER_NODE / 1024 / 1024 << "MiB" << " + "
           << graph.base_graph().edge_count() * steiner_graph::SIZE_PER_EDGE / 1024 / 1024 << "MiB"
           << std::endl;

    double vm, res;
    process_mem_usage(vm, res);
    output << "    actual memory usage with graph loaded: VM " << vm / 1024 << "MiB, RES " << res / 1024 << "MiB"
           << std::endl;
}

template<typename GraphT>
void Graph::GraphImplementation<GraphT>::write_graph_stats(std::ostream &output) const {
    output << "\r\agraph has "
           << std::setw(12) << graph.node_count() << " nodes and "
           << std::setw(12) << graph.edge_count() / 2 << " edges" << std::endl;

    double vm, res;
    process_mem_usage(vm, res);
    output << "memory usage with graph loaded: VM " << vm / 1024 << "MiB, RES " << res / 1024 << "MiB"
           << std::endl;
}


template<typename GraphT>
coordinate_t Graph::GraphImplementation<GraphT>::node_coordinates(int node_id) const {
    return graph.node(node_id).coordinates;
}

template<typename GraphT>
int Graph::GraphImplementation<GraphT>::edge_count() const {
    return graph.edge_count();
}

template<typename GraphT>
int Graph::GraphImplementation<GraphT>::node_count() const {
    return graph.node_count();
}


template<typename GraphT>
void Graph::GraphImplementation<GraphT>::write_graph_file(std::string path) const {
    std::ofstream output(path);
    if (path.ends_with(".fmi"))
        fmi_file_io::write(output, graph);
    else if (path.ends_with(".steiner.graph"))
        fmi_file_io::write(output, graph);
    else if (path.ends_with(".graph"))
        triangulation_file_io::write(output, graph);
    else if (path.ends_with(".gl"))
        gl_file_io::write(output, graph);

    output.close();
}


template<typename GraphT>
void Graph::GraphImplementation<GraphT>::project(Projection projection) {
    for (int i = 0; i < graph.node_count(); ++i) {
        project_coordinate(graph.node(i).coordinates, projection);
    }
}

template<>
void Graph::GraphImplementation<steiner_graph>::project(Projection projection) {
    for (int i = 0; i < graph.base_graph().node_count(); ++i) {
        project_coordinate(graph.node(i).coordinates, projection);
    }
}


template<typename... Args>
Graph Graph::read_graph_file(std::string path, double epsilon, Args... args) {
    std::ifstream input(path);

    if (path.ends_with(".graph") && epsilon != 0.0)
        return { GraphImplementation<steiner_graph>( triangulation_file_io::read_steiner(input, epsilon))};
    else if (path.ends_with(".graph"))
        return { GraphImplementation<std_graph_t>( triangulation_file_io::read<std_graph_t>(input))};
    else if (path.ends_with(".fmi"))
        return { GraphImplementation<std_graph_t>(fmi_file_io::read<std_graph_t>(input)) };
    else if (path.ends_with(".sch"))
        return {GraphImplementation<ch_graph_t>(fmi_file_io::read<ch_graph_t>(input))};
    else if (path.ends_with(".gl"))
        return {GraphImplementation<gl_graph_t>(fmi_file_io::read<gl_graph_t>(input)) };
    else
        throw std::invalid_argument("unrecognized file ending");

// ...

    input.close();
}


template<typename... Args>
Graph Graph::read_graph_file(std::string path, Args... args) {
    std::ifstream input(path);

    if (path.ends_with(".graph")) {
        return { GraphImplementation<steiner_graph>( triangulation_file_io::read_steiner(input, 0.5F))};
    } else if (path.ends_with(".fmi"))
        return { GraphImplementation<std_graph_t>(fmi_file_io::read<std_graph_t>(input)) };
    else if (path.ends_with(".sch"))
        return {GraphImplementation<ch_graph_t>(fmi_file_io::read<ch_graph_t>(input))};
    else if (path.ends_with(".gl"))
        return {GraphImplementation<gl_graph_t>(fmi_file_io::read<gl_graph_t>(input)) };
    else
        throw std::invalid_argument("unrecognized file ending");

// ...

        input.close();
}

#pragma once

#include "Graph.h"
#include "Query.h"

#include "../util/memory_usage_impl.h"
#include "../routing_impl.h"
#include "Statistics.h"


template<typename GraphT>
GraphType Graph::GraphImplementation<GraphT>::type() const {
    return GraphType::STD_GRAPH;
}

template<>
GraphType Graph::GraphImplementation<steiner_graph<true>>::type() const {
    return GraphType::STEINER_GRAPH_COORDS_EXPLICIT;
}

template<>
GraphType Graph::GraphImplementation<steiner_graph<false>>::type() const {
    return GraphType::STEINER_GRAPH_IMPLICIT;
}


template<typename GraphT>
void Graph::GraphImplementation<GraphT>::write_graph_stats(std::ostream &output) const {
    if constexpr (SteinerGraph<Graph>) {
        output << "\r\agraph has "
               << std::setw(16) << graph->node_count() << " nodes and "
               << std::setw(16) << graph->edge_count() << " edges"
               << "\n     with "
               << std::setw(16) << graph->base_graph().node_count() << " nodes and "
               << std::setw(16) << graph->base_graph().edge_count() << " edges stored explicitly (ε = "
               << graph->epsilon() << ")" << std::endl;

        output << "expected size per node: "
               << std::setw(5) << GraphT::SIZE_PER_NODE << " and per edge "
               << std::setw(5) << GraphT::SIZE_PER_EDGE << " -> "
               << graph->base_graph().node_count() * GraphT::SIZE_PER_NODE / 1024 / 1024 << "MiB" << " + "
               << graph->base_graph().edge_count() * GraphT::SIZE_PER_EDGE / 1024 / 1024 << "MiB"
               << std::endl;

        double vm, res;
        process_mem_usage(vm, res);
        output << "    actual memory usage with graph loaded: VM " << vm / 1024 << "MiB, RES " << res / 1024 << "MiB"
               << std::endl;
    } else {
        output << "\r\agraph has "
               << std::setw(12) << graph->node_count() << " nodes and "
               << std::setw(12) << graph->edge_count() / 2 << " edges" << std::endl;

        double vm, res;
        process_mem_usage(vm, res);
        output << "memory usage with graph loaded: VM " << vm / 1024 << "MiB, RES " << res / 1024 << "MiB"
               << std::endl;
    }
}

template<typename GraphT>
void Graph::GraphImplementation<GraphT>::write_graph_stats(table &out) const {
    out.put(Statistics::GRAPH_FILE, _graph_file);

    if constexpr (SteinerGraph<GraphT>) {
        out.put(Statistics::NODE_COUNT, graph->node_count());
        out.put(Statistics::EDGE_COUNT, graph->edge_count());

        out.put(Statistics::EPSILON, graph->epsilon());

        out.put(Statistics::STORED_NODE_COUNT, graph->base_graph().node_count());
        out.put(Statistics::STORED_EDGE_COUNT, graph->base_graph().edge_count());
    } else {
        out.put(Statistics::NODE_COUNT, graph->node_count());
        out.put(Statistics::EDGE_COUNT, graph->edge_count());

        out.put(Statistics::EPSILON, 0);

        out.put(Statistics::STORED_NODE_COUNT, graph->node_count());
        out.put(Statistics::STORED_EDGE_COUNT, graph->edge_count());
    }
}

template<typename GraphT>
coordinate_t Graph::GraphImplementation<GraphT>::node_coordinates(long node_id) const {
    return graph->node(node_id).coordinates;
}

template<typename GraphT>
long Graph::GraphImplementation<GraphT>::node_at(coordinate_t &position) const {
    if constexpr (SteinerGraph<GraphT>) {
        for (auto &&id: graph->base_graph().node_ids()) {
            auto &&node = graph->node(id);
            if (node.coordinates == position)
                return id;
        }

        for (auto &&id: graph->node_ids()) {
            auto &&node = graph->node(id);
            if (node.coordinates == position)
                return graph->base_graph().node_count() - 1;
        }

        return optional::none_value<long>;
    } else {
        for (auto &&id: graph->node_ids()) {
            auto &&node = graph->node(id);
            if (node.coordinates == position)
                return id;
        }
        return optional::none_value<long>;
    }
}

template<typename GraphT>
std::size_t Graph::GraphImplementation<GraphT>::edge_count() const {
    return graph->edge_count();
}

template<typename GraphT>
std::size_t Graph::GraphImplementation<GraphT>::node_count() const {
    return graph->node_count();
}

template<typename GraphT>
std::string& Graph::GraphImplementation<GraphT>::graph_file() {
    return _graph_file;
}


template<typename GraphT>
void Graph::GraphImplementation<GraphT>::write_graph_file(std::string path) const {
    std::ofstream output(path);

    if (path.ends_with(".fmi"))
        fmi_file_io::write(output, *graph);
    else if (path.ends_with(".steiner.fmi"))
        fmi_file_io::write(output, *graph);
    else if (path.ends_with(".graph"))
        triangulation_file_io::write(output, *graph);
    else if (path.ends_with(".gl"))
        gl_file_io::write(output, *graph);

    output.close();
}

template<typename GraphT>
void Graph::GraphImplementation<GraphT>::write_graph_file(std::string path, int color, int linewidth) const {
    std::ofstream output(path);

    if (path.ends_with(".gl"))
        gl_file_io::write(output, *graph, linewidth, color);
    else
        throw std::invalid_argument("This overload can only be used for writing .gl files");

    output.close();
}


template<>
void Graph::GraphImplementation<gl_graph_t>::write_subgraph_file(std::string path, coordinate_t bottom_left,
                                                                 coordinate_t top_right) const {
    std::ofstream output(path);

    auto &&all_nodes = graph->node_ids();
    std::vector<gl_graph_t::node_id_type> nodes;
    std::vector<gl_graph_t::edge_id_type> edges;

    for (auto node_id: all_nodes) {
        if (!is_in_rectangle(graph->node(node_id).coordinates, bottom_left, top_right))
            continue;
        if (nodes.size() > 1000000 || edges.size() > 200000000)
            break;
        nodes.emplace_back(node_id);

        for (auto edge: graph->outgoing_edges(node_id)) {
            if (!is_in_rectangle(graph->node(edge.destination).coordinates, bottom_left, top_right))
                continue;

            edges.emplace_back(graph->edge_id(node_id, edge.destination));
        }
    }

    gl_graph_t::subgraph_type s(std::move(nodes), std::move(edges));
    gl_graph_t subgraph = gl_graph_t::make_graph(*graph, s);
    if (path.ends_with(".fmi")) {
        fmi_file_io::write(output, subgraph);
    } else if (path.ends_with(".gl")) {
        gl_file_io::write(output, subgraph);
    } else if (path.ends_with(".graph")) {
        throw std::runtime_error("Not implemented");
    }
}

template<typename GraphT>
void Graph::GraphImplementation<GraphT>::write_subgraph_file(std::string path, coordinate_t bottom_left,
                                                             coordinate_t top_right) const {
    std::ofstream output(path);

    if constexpr (SteinerGraph<GraphT>) {
        std::vector<typename GraphT::base_topology_type::node_id_type> nodes;
        std::vector<typename GraphT::base_topology_type::edge_id_type> edges;

        auto &&all_nodes = graph->base_graph().node_ids();
        for (auto &&node_id: all_nodes) {
            if (!is_in_rectangle(graph->node(node_id).coordinates, bottom_left, top_right))
                continue;
            if (nodes.size() > 1000000 || edges.size() > 200000000)
                break;
            nodes.emplace_back(node_id);

            for (auto &&edge: graph->base_graph().outgoing_edges(node_id)) {
                if (!is_in_rectangle(graph->node(edge.destination).coordinates, bottom_left, top_right))
                    continue;

                edges.emplace_back(graph->base_graph().edge_id(node_id, edge.destination));
            }
        }

        subgraph<typename GraphT::base_topology_type> s(std::move(nodes), std::move(edges));
        auto subgraph = GraphT::make_graph(*graph, s);
        if (path.ends_with(".fmi")) {
            fmi_file_io::write(output, subgraph);
        } else if (path.ends_with(".gl")) {
            gl_file_io::write(output, subgraph);
        } else if (path.ends_with(".graph")) {
            throw std::runtime_error("Not implemented");
        }
    } else {
        std::vector<typename GraphT::node_id_type> nodes;
        std::vector<typename GraphT::edge_id_type> edges;

        auto &&all_nodes = graph->node_ids();
        for (auto node_id: all_nodes) {
            if (!is_in_rectangle(graph->node(node_id).coordinates, bottom_left, top_right))
                continue;
            if (nodes.size() > 1000000 || edges.size() > 200000000)
                break;
            nodes.emplace_back(node_id);

            for (auto edge: graph->outgoing_edges(node_id)) {
                if (!is_in_rectangle(graph->node(edge.destination).coordinates, bottom_left, top_right))
                    continue;

                edges.emplace_back(graph->edge_id(node_id, edge.destination));
            }
        }

        typename GraphT::subgraph_type s(std::move(nodes), std::move(edges));
        std_graph_t subgraph = std_graph_t::make_graph(*graph, s);
        if (path.ends_with(".fmi")) {
            fmi_file_io::write(output, subgraph);
        } else if (path.ends_with(".gl")) {
            gl_file_io::write(output, subgraph);
        } else if (path.ends_with(".graph")) {
            throw std::runtime_error("Not implemented");
        }
    }
}


template<typename GraphT>
void Graph::GraphImplementation<GraphT>::project(Projection projection) {
    if constexpr (SteinerGraph<GraphT>) {
        for (std::size_t i = 0; i < graph->base_graph().node_count(); ++i) {
            project_coordinate(graph->node(i).coordinates, projection);
        }
    } else {
        for (std::size_t i = 0; i < graph->node_count(); ++i) {
            project_coordinate(graph->node(i).coordinates, projection);
        }
    }
}

template<typename... Args>
void Graph::read_graph_file(std::string path, RoutingConfiguration const &config, double epsilon, Args... args) {
    std::ifstream input(path);

    if (input.fail()) {
        throw std::invalid_argument("graph file can't be read from");
    }

    if (path.ends_with(".graph") && epsilon != 0.0) {
        if (config.store_coords) {
            impl = std::make_unique<GraphImplementation < steiner_graph<true>> >
                   (triangulation_file_io::read_steiner<steiner_graph<true> >(input, epsilon));
        } else {
            impl = std::make_unique<GraphImplementation < steiner_graph<false>> >
                   (triangulation_file_io::read_steiner<steiner_graph<false>>(input, epsilon));
        }
    } else if (path.ends_with(".graph")) {
        impl = std::make_unique<GraphImplementation < std_graph_t>>
        (triangulation_file_io::read<std_graph_t>(input));
    } else if (path.ends_with(".fmi")) {
        impl = std::make_unique<GraphImplementation < std_graph_t>>
        (fmi_file_io::read<std_graph_t>(input));
    } else if (path.ends_with(".sch")) {
        impl = std::make_unique<GraphImplementation < ch_graph_t>>
        (fmi_file_io::read<ch_graph_t>(input));
    } else if (path.ends_with(".gl")) {
        impl = std::make_unique<GraphImplementation < gl_graph_t>>
        (fmi_file_io::read<gl_graph_t>(input));
    }
        // ...
    else
        throw std::invalid_argument("unrecognized file ending");

    impl->graph_file() = path;
}


template<typename... Args>
void Graph::read_graph_file(std::string path, RoutingConfiguration const &config, Args... /*args*/) {
    std::ifstream input(path);

    if (input.fail()) {
        throw std::invalid_argument("graph file can't be read from");
    }

    if (path.ends_with(".graph")) {
        if (config.store_coords) {
            impl = std::make_unique<GraphImplementation < steiner_graph<true>> >
                   (triangulation_file_io::read_steiner<steiner_graph<true>>(input, 0.5F));
        } else {
            impl = std::make_unique<GraphImplementation < steiner_graph<false>> >
                   (triangulation_file_io::read_steiner<steiner_graph<false>>(input, 0.5F));
        }
    } else if (path.ends_with(".fmi"))
        impl = std::make_unique<GraphImplementation < std_graph_t>>
    (fmi_file_io::read<std_graph_t>(input));
    else if (path.ends_with(".sch"))
        impl = std::make_unique<GraphImplementation < ch_graph_t>>
    (fmi_file_io::read<ch_graph_t>(input));
    else if (path.ends_with(".gl"))
        impl = std::make_unique<GraphImplementation < gl_graph_t>>
    (fmi_file_io::read<gl_graph_t>(input));
    // ...
    else
    throw std::invalid_argument("unrecognized file ending");

    impl->graph_file() = path;
}

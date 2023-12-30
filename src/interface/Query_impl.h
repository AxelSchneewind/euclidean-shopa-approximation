#pragma once

#include <chrono>
#include <iostream>

#include "../graph/base_types.h"

#include "../graph/graph.h"
#include "Graph.h"

template <typename GraphT, typename RouterT>
Result::Result(GraphT const& graph, QueryImplementation<GraphT> query, RouterT const& router, std::chrono::duration<double, std::milli> duration)
        : pimpl(graph, std::move(query), router, duration) {};

template<RoutableGraph GraphT>
std_graph_t make_beeline(GraphT const& graph, typename GraphT::node_id_type from, typename GraphT::node_id_type to) {
     std::vector<node_t> nodes { graph.node(from), graph.node(to)};
     unidirectional_adjacency_list<int, edge_t>::adjacency_list_builder edges(2);
     edges.add_edge(0, 1, {distance(nodes[0].coordinates, nodes[1].coordinates)});
     std_graph_t beeline = std_graph_t::make_graph(std::move(nodes), adjacency_list<int, edge_t>::make_bidirectional(edges.get()));
     return beeline;
}

template<RoutableGraph GraphT>
QueryImplementation<GraphT>::QueryImplementation(GraphT const&graph, long from, long to)
: _from(from)
, _to(to)
, _from_internal(from)
, _to_internal(to)
, _beeline_distance(distance(graph.node(_from_internal).coordinates, graph.node(_to_internal).coordinates))
, _beeline(make_beeline(graph, _from_internal, _to_internal))
{};

template<>
QueryImplementation<steiner_graph>::QueryImplementation(steiner_graph const&graph, long from, long to)
: _from(from)
, _to(to)
, _from_internal(graph.from_base_node_id(from))
, _to_internal(graph.from_base_node_id(to))
, _beeline_distance(distance(graph.node(_from_internal).coordinates, graph.node(_to_internal).coordinates))
, _beeline(make_beeline(graph, _from_internal, _to_internal))
{ };

template<RoutableGraph GraphT>
QueryImplementation<GraphT>::QueryImplementation() : _from_internal{none_value<node_id_type>}, _to_internal{none_value<node_id_type>}, _from{none_value<long>}, _to{none_value<long> } {}
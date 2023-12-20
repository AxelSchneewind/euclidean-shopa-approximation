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
QueryImplementation<GraphT>::QueryImplementation(GraphT const&graph, long from, long to) : _from(from), _to(to), _from_internal(from), _to_internal(to){ };

template<>
QueryImplementation<steiner_graph>::QueryImplementation(steiner_graph const&graph, long from, long to) : _from(from), _to(to), _from_internal(graph.from_base_node_id(from)), _to_internal(graph.from_base_node_id(to)) { };

template<RoutableGraph GraphT>
QueryImplementation<GraphT>::QueryImplementation() : _from_internal{none_value<node_id_type>}, _to_internal{none_value<node_id_type>}, _from{none_value<long>}, _to{none_value<long> } {}

#pragma once

#include "dijkstra.h"

#include "../graph/base_types.h"
#include "../util/contract.h"

#include <cstddef>


template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
typename L::value_type dijkstra<G, Q, L, N, Heuristic>::get_label(node_id_type node) const { return _labels->at(node); }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
L &dijkstra<G, Q, L, N, Heuristic>::labels() { return *_labels; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
const N &dijkstra<G, Q, L, N, Heuristic>::neighbors() const { return _neighbors; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
const L &dijkstra<G, Q, L, N, Heuristic>::labels() const { return *_labels; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
Q &dijkstra<G, Q, L, N, Heuristic>::queue() { return _queue; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
const Q &dijkstra<G, Q, L, N, Heuristic>::queue() const { return _queue; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
dijkstra<G, Q, L, N, Heuristic>::node_id_type dijkstra<G, Q, L, N, Heuristic>::target() const { return _target_node; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
dijkstra<G, Q, L, N, Heuristic>::node_id_type dijkstra<G, Q, L, N, Heuristic>::source() const { return _start_node; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
dijkstra<G, Q, L, N, Heuristic>::dijkstra(std::shared_ptr<G> graph, Q &&queue, L &&labels, N &&neighbors)
        : _graph(graph)
        , _queue(std::move(queue))
        , _labels(std::make_shared<L>(std::move(labels)))
        , _neighbors(std::move(neighbors)) {
    queue.clear();
}


template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
dijkstra<G, Q, L, N, Heuristic>::dijkstra(std::shared_ptr<G> graph)
        : _graph(std::move(graph))
        , _labels{std::make_shared<L>(_graph, optional::none_value<typename L::value_type>)}
        , _start_node{optional::none_value<node_id_type>}
        , _target_node{optional::none_value<node_id_type>}
        , _queue{_graph, _labels}
        , _neighbors{_graph, _labels}
        , _heuristic{_graph, _labels}
        {}



template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, typename Heuristic>
dijkstra<G, Q, L, N, Heuristic>::node_cost_pair_type
dijkstra<G, Q, L, N, Heuristic>::current() const {
    // assert(!_queue.empty());
    // TODO remove this check and pass reference
    return _queue.empty() ? optional::none_value<dijkstra<G, Q, L, N, Heuristic>::node_cost_pair_type> : _queue.top();
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, typename Heuristic>
bool
dijkstra<G, Q, L, N, Heuristic>::reached(G::node_id_type node) const {
    static_assert(  (HasHeuristic<typename L::value_type> && HasHeuristic<typename Q::value_type>)
                 || (HasDistance<typename L::value_type> && HasDistance<typename Q::value_type>));
    if constexpr (HasHeuristic<typename L::value_type> && HasHeuristic<typename Q::value_type>) {
        return _labels->contains(node) && _labels->at(node).heuristic() < current().heuristic();
    } else if constexpr (HasDistance<typename L::value_type> && HasDistance<typename Q::value_type>) {
        return _labels->contains(node) && _labels->at(node).distance() < current().distance();
    }
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, typename Heuristic>
bool
dijkstra<G, Q, L, N, Heuristic>::queue_empty() const {
    return _queue.empty();
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, typename Heuristic>
void
dijkstra<G, Q, L, N, Heuristic>::init(node_id_type start_node, node_id_type target_node) {
    _target_node = target_node;
    _start_node = start_node;

    // call init on each member that has it
    if constexpr (HasInit<Q, node_id_type>) {
        _queue.init(start_node, target_node);
    }
    if constexpr (HasInit<L, node_id_type>) {
        _labels->init(start_node, target_node);
    }
    if constexpr (HasInit<N, node_id_type>) {
        _neighbors.init(start_node, target_node);
    }
    if constexpr (HasInit<Heuristic, node_id_type>) {
        _heuristic.init(start_node, target_node);
    }

    // add start node to queue
    if (!optional::is_none(start_node)) {
        typename Q::value_type ncp{};
        ncp.node() = start_node;
        if constexpr (HasPredecessor<typename Q::value_type>) {
            ncp.predecessor() = start_node;
        }
        if constexpr (HasDistance<typename Q::value_type>) {
            ncp.distance() = 0.0;
        }
        if constexpr (HasHeuristic<typename Q::value_type>) {
            ncp.heuristic() = 0.0;
            _heuristic(ncp);
        }
        _queue.push(ncp);

        auto& label = (*_labels)[start_node];
        label = ncp;
        if constexpr (HasPredecessor<typename L::value_type>) {
            label.predecessor() = start_node;
        }
        if constexpr (HasDistance<typename L::value_type>) {
            label.distance() = 0.0;
        }
    }

    _pull_count = 0;
    _push_count = 0;
    _edges_checked = 0;
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, typename Heuristic>
void
dijkstra<G, Q, L, N, Heuristic>::expand(node_cost_pair_type node) {
    static std::vector<node_cost_pair_type> node_cost_pairs(128);
    static std::vector<coordinate_t > coordinates(128);
    node_cost_pairs.clear();
    coordinates.clear();

    assert(!optional::is_none(node.node()));
    // assert(node.node() == _start_node || node.node() != node.predecessor());

    //
    static constexpr bool geometric_heuristic = !std::is_same_v<Heuristic, void> && requires (Heuristic h) { h(node_cost_pairs, coordinates); };
    static constexpr bool geometric_neighbors = requires(N n, std::vector<node_cost_pair_type> out, std::vector<coordinate_t> coords_out) { n(out, coords_out);};
    if constexpr (geometric_heuristic && geometric_neighbors) {
        _neighbors(node, node_cost_pairs, coordinates);
    } else {
        _neighbors(node, node_cost_pairs);
    }

    _edges_checked += node_cost_pairs.size();

    // filter which neighbors are improved
    size_t to_index = 0;
    for (size_t i = 0; i < node_cost_pairs.size(); i++) {
        node_cost_pair_type &successor = node_cost_pairs[i];
        node_id_type const  &successor_node = successor.node();

        assert(!optional::is_none(successor_node));
        assert(successor.distance() > 0);

        distance_t const& successor_cost = (*_labels)[successor_node].distance(); // use shortest distance
        distance_t const& new_cost = successor.distance();

        assert(new_cost >= node.distance());
        if (new_cost < successor_cost) [[likely]] {
            // assert(successor_node != node.predecessor());

            // (re-)insert node into the queue with updated priority
            if constexpr (geometric_neighbors && geometric_heuristic) {
                coordinates[to_index] = coordinates[i];
            }
            node_cost_pairs[to_index++] = successor;
        }
    }
    node_cost_pairs.resize(to_index);

    // if heuristic requires coordinates and neighbor finder doesnt return them, get them now
    if constexpr (geometric_heuristic && !geometric_neighbors) {
        coordinates.resize(to_index);
        for (size_t i = 0; i < node_cost_pairs.size(); ++i) {
            coordinates[i] = _graph->node_coordinates(node_cost_pairs[i].node());
        }
    }

    // apply heuristic if given
    if constexpr (!std::is_same_v<Heuristic, void> && geometric_heuristic) {
        for (size_t i = 0; i < node_cost_pairs.size(); ++i) {
            _heuristic(node_cost_pairs[i], coordinates[i]);
        }
    } else if constexpr (!std::is_same_v<Heuristic, void>) {
        for (size_t i = 0; i < node_cost_pairs.size(); ++i) {
            _heuristic(node_cost_pairs[i]);
        }
    }

    // store label values
    for (node_cost_pair_type & successor : node_cost_pairs) {
        auto& label = _labels->at(successor.node());

        // TODO write assign operation for node_info
        if constexpr (HasDistance<typename Q::value_type>)
            label.distance() = successor.distance();
        else
            label.distance() = _labels->at(node.node()).distance() + distance(_graph->node_coordinates(successor.node()), _graph->node_coordinates(node.node()));

        if constexpr (HasPredecessor<typename L::value_type>)
            label.predecessor() = node.node();
        if constexpr (HasPredecessor<typename Q::value_type>)
            successor.predecessor() = node.node();

        if constexpr (HasHeuristic<typename L::value_type> && HasHeuristic<typename Q::value_type>)
            label.heuristic() = successor.heuristic();
        if constexpr (HasHeuristic<typename L::value_type> && !HasHeuristic<typename Q::value_type>)
            label.heuristic() = successor.distance();
    }

    // push all
    _queue.push_range(node_cost_pairs);
    _push_count += node_cost_pairs.size();
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, typename Heuristic>
void
dijkstra<G, Q, L, N, Heuristic>::step() {
    node_cost_pair_type ncp = current();

    // expand to adjacent nodes
    expand(ncp);

    // remove current node
    _queue.pop();
    _pull_count++;

    // remove already labelled nodes
    while (!_queue.empty() && _labels->at(_queue.top().node()).distance() < _queue.top().distance()) [[likely]] {
        _queue.pop();
    }
}


template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
G::path_type dijkstra<G, Q, L, N, Heuristic>::path(node_id_type target) const {
    typename G::node_id_type fwd_node = target;

    auto result = std::vector<typename G::node_id_type>();
    result.push_back(fwd_node);

    if (!reached(target))
        return {std::move(result)};


    while (!optional::is_none(fwd_node) && fwd_node != _start_node) {
        fwd_node = get_label(fwd_node).predecessor();

        if (optional::is_none(fwd_node)) break;

        result.push_back(fwd_node);
    }

    for (size_t i = 0; i < result.size() / 2; ++i) {
        std::swap(result[i], result[result.size() - 1 - i]);
    }

    remove_duplicates_sorted(result);

    return {std::move(result)};
}

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
G::subgraph_type dijkstra<G, Q, L, N, Heuristic>::shortest_path_tree(std::size_t max_node_count) const {
    std::vector<typename G::node_id_type> nodes;
    std::vector<typename G::edge_id_type> edges;

    if (!reached(_start_node))
        return {std::move(nodes), std::move(edges)};

    // add nodes and edges that have been visited
    auto &&visited = _labels->all_visited();
    for (auto const &node_id: visited) {
        if (nodes.size() >= max_node_count || edges.size() >= max_node_count)
            break;

        if (!reached(node_id))
            continue;

        nodes.emplace_back(node_id);

        auto predecessor = _labels->at(node_id).predecessor();
        if (optional::is_none(predecessor) || predecessor == node_id)
            continue;

        auto&& edge = _graph->topology().edge_id(predecessor, node_id);
        edges.emplace_back(edge);
    }

    remove_duplicates(nodes);
    remove_duplicates(edges);

    typename G::subgraph_type subgraph{std::move(nodes), std::move(edges)};
    filter_nodes(subgraph, [&](auto const &node) -> bool { return get_label(node).value() < current().value(); });

    return subgraph;
}

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
std::size_t dijkstra<G, Q, L, N, Heuristic>::edges_checked() const { return _edges_checked; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
std::size_t dijkstra<G, Q, L, N, Heuristic>::pull_count() const { return _pull_count; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, typename Heuristic>
std::size_t dijkstra<G, Q, L, N, Heuristic>::push_count() const { return _push_count; }

#pragma once

#include "base_types.h"
#include <algorithm>
#include <cassert>
#include <span>
#include <vector>
using std::vector;

template<typename E>
struct adjacency_list_edge_t {
    node_id_t source;
    node_id_t destination;
    E info;

    adjacency_list_edge_t();
    adjacency_list_edge_t(node_id_t source, node_id_t destination, E info);

    adjacency_list_edge_t invert(const std::vector<edge_id_t> &new_edge_indices) const;
    bool operator==(const adjacency_list_edge_t &other);
};

// class that stores edges (topology) of a graph
template<typename E>
class adjacency_list {
public:
    class adjacency_list_builder {
    private:
        size_t nodecount;
        size_t edgecount;

        vector<adjacency_list_edge_t<E>> edges;

    public:
        adjacency_list_builder() : nodecount(0), edgecount(0) {};

        void add_node(const node_id_t& node);
        void add_edge(const adjacency_list_edge_t<E>& edge) { edges.push_back(edge); };
        void add_edge(const node_id_t &source, const node_id_t &destination, const E &info);

        adjacency_list<E> get();
    };

private:
    size_t nodecount;
    size_t edgecount;
    vector<edge_id_t> offsets;
    vector<adjacency_list_edge_t<E>> edges;

public:
    ~adjacency_list();

    adjacency_list(node_id_t node_count, edge_id_t edge_count);

    adjacency_list(node_id_t node_count, edge_id_t edge_count, std::vector<edge_id_t> &&offsets, vector<adjacency_list_edge_t<E>> &&edges);

    // move constructor
    adjacency_list(adjacency_list &&other) noexcept;

    //adjacency_list<E> inverse() const;
    adjacency_list<E> inverse(std::vector<node_id_t>& forward_ids, std::vector<node_id_t>& backward_ids) const;

    // access lists directly when not const
    vector<edge_id_t> &offset_list();
    vector<adjacency_list_edge_t<E>> &edge_list();

    // TODO remove
    void push_back_node(node_id_t node_index);
    void push_back_edge(const adjacency_list_edge_t<E> &edge);
    void push_back_edge(const node_id_t &source, const node_id_t &destination, const E &info);

    // NOT TESTED
    void add_edge(const adjacency_list_edge_t<E> &edge);
    void add_edge(const node_id_t &source, const node_id_t &destination, const E &info);
    void remove_edge(node_id_t &&source, node_id_t &&destination);

    inline const size_t node_count() const;
    inline const size_t edge_count() const;

    inline edge_id_t &offset(const node_id_t &node);
    inline edge_id_t &offset_next(const node_id_t &node);

    inline const node_id_t &source(const edge_id_t &edge) const;
    inline const node_id_t &destination(const edge_id_t &edge) const;

    inline const E &edge(const edge_id_t &edge) const;
    inline const adjacency_list_edge_t<E> &adjlist_edge(const edge_id_t &edge) const;

    inline edge_id_t edge_index(const node_id_t &source, const node_id_t &dest) const;
    inline std::span<const adjacency_list_edge_t<E>, std::dynamic_extent> node_edges(const node_id_t &node) const;

    inline const edge_id_t &offset(const node_id_t &node) const;
    inline const edge_id_t &offset_next(const node_id_t &node) const;


    bool operator==(const adjacency_list<E> &other);
};
template<typename E>
const adjacency_list_edge_t<E> &
adjacency_list<E>::adjlist_edge(const edge_id_t &edge) const {
    return edges[edge];
}
template<typename E>
void
adjacency_list<E>::adjacency_list_builder::add_node(const node_id_t &node) {
    if (nodecount <= node)
        nodecount = node + 1;
}
template<typename E>
void
adjacency_list<E>::adjacency_list_builder::add_edge(const node_id_t &source, const node_id_t &destination, const E &info) {
    add_node(source);
    add_node(destination);

    auto edge = adjacency_list_edge_t<E>(source, destination, info);
    edges.push_back(edge);
    edgecount++;
}
template<typename E>
adjacency_list<E>
adjacency_list<E>::adjacency_list_builder::get() {
    edgecount = edges.size();

    // order by source id
    auto order = [&] (adjacency_list_edge_t<E> e1, adjacency_list_edge_t<E> e2) { return e1.source < e2.source; };
    std::sort(edges.begin(), edges.end(), order);

    // make offset array
    int index = 0;
    std::vector<edge_id_t> offset;
    offset.push_back(0);
    for (auto e : edges) {
        while(offset.size() < e.source)
            offset.push_back(index);
        index++;
    }

    while(offset.size() <= nodecount)
        offset.push_back(edgecount);

    auto result = adjacency_list<E>(nodecount, edgecount, std::move(offset), std::vector(edges));
    return result;
}


template<typename E>
inline const size_t
adjacency_list<E>::node_count() const { return nodecount; }
template<typename E>
inline const size_t
adjacency_list<E>::edge_count() const { return edgecount; }

template<typename E>
inline edge_id_t &
adjacency_list<E>::offset(const node_id_t &node) { return offsets[node]; }
template<typename E>
inline edge_id_t &
adjacency_list<E>::offset_next(const node_id_t &node) { return offsets[node + 1]; }

template<typename E>
inline const node_id_t &
adjacency_list<E>::source(const edge_id_t &edge) const { return edges[edge].source; }
template<typename E>
inline const node_id_t &
adjacency_list<E>::destination(const edge_id_t &edge) const { return edges[edge].destination; }

template<typename E>
inline edge_id_t
adjacency_list<E>::edge_index(const node_id_t &source, const node_id_t &dest) const {
    edge_id_t idx = offsets[source];
    while (idx < offsets[source + 1] && destination(idx) != dest)
        idx++;

    return idx;
}

template<typename E>
inline std::span<const adjacency_list_edge_t<E>, std::dynamic_extent>
adjacency_list<E>::node_edges(const node_id_t &node) const {
    auto first = &edges[offset(node)];
    auto last  = &edges[offset(node + 1)];
    return std::span(first, last);
}

template<typename E>
inline const edge_id_t &
adjacency_list<E>::offset(const node_id_t &node) const { return offsets[node]; }
template<typename E>
inline const edge_id_t &
adjacency_list<E>::offset_next(const node_id_t &node) const { return offsets[node + 1]; }

template<typename E>
inline const E &
adjacency_list<E>::edge(const edge_id_t &edge) const { return edges[edge].info; }



template<typename E>
adjacency_list_edge_t<E>::adjacency_list_edge_t() = default;

template<typename E>
adjacency_list_edge_t<E>::adjacency_list_edge_t(node_id_t source, node_id_t destination, E info) : source(source), destination(destination), info(info) {}

template<typename E>
adjacency_list_edge_t<E>
adjacency_list_edge_t<E>::invert(const std::vector<edge_id_t> &new_edge_indices) const {
    return adjacency_list_edge_t(destination, source, info.invert(new_edge_indices));
}

template<typename E>
bool
adjacency_list_edge_t<E>::operator==(const adjacency_list_edge_t &other) { return (source == other.source && destination == other.destination && info == other.info); }

template<typename E>
adjacency_list<E>::~adjacency_list() {
    offsets.clear();
    offsets.shrink_to_fit();
    edges.clear();
    edges.shrink_to_fit();
    offsets.emplace_back(0);
}

template<typename E>
adjacency_list<E>::adjacency_list(node_id_t node_count, edge_id_t edge_count)
    : nodecount(0), edgecount(0), offsets(), edges() {
    offsets.reserve(nodecount + 1);
    edges.reserve(edgecount);

    offsets.emplace_back(0);
}

template<typename E>
adjacency_list<E>::adjacency_list(node_id_t node_count, edge_id_t edge_count, std::vector<edge_id_t> &&offsets, vector<adjacency_list_edge_t<E> > &&edges)
    : nodecount(node_count), edgecount(edge_count), offsets(std::move(offsets)), edges(std::move(edges)) {
    if (this->offsets.size() < node_count)// given array is too small
        throw;
    else if (this->offsets.size() == node_count)// array is missing trailing offset entry
        this->offsets.push_back(edge_count);
    else
        this->offsets.back() = edge_count;
}

template<typename E>
adjacency_list<E>::adjacency_list(adjacency_list &&other) noexcept
    : nodecount(other.nodecount),
      edgecount(other.edgecount),
      offsets(std::move(other.offsets)),
      edges(std::move(other.edges)) {
}

// access lists directly when not const
template<typename E>
vector<edge_id_t> &
adjacency_list<E>::offset_list() { return offsets; }
template<typename E>
vector<adjacency_list_edge_t<E> > &
adjacency_list<E>::edge_list() { return edges; }

template<typename E>
void
adjacency_list<E>::push_back_node(node_id_t node_index) {
    offsets.pop_back();// pop off last entry
    while (offsets.size() <= node_index) {
        // new node introduced
        offsets.push_back(edge_count());
        nodecount++;
    }
    offsets.push_back(edge_count());
}

template<typename E>
void
adjacency_list<E>::push_back_edge(const adjacency_list_edge_t<E> &edge) {
    push_back_node(edge.source);

    edges.push_back(edge);

    edgecount++;
    offsets.back() = edge_count();
}
template<typename E>
void
adjacency_list<E>::push_back_edge(const node_id_t &source, const node_id_t &destination, const E &info) {
    push_back_edge(adjacency_list_edge_t(source, destination, info));
}

// NOT TESTED
template<typename E>
void
adjacency_list<E>::add_edge(const adjacency_list_edge_t<E> &edge) {
    add_edge(edge.source, edge.destination, edge.info);
}
template<typename E>
void
adjacency_list<E>::add_edge(const node_id_t &source, const node_id_t &destination, const E &info) {
    for (node_id_t node_index = node_count(); node_index < source; node_index++)
        offsets.push_back(edge_count());

    if (source < node_count())
        edges.insert(edges.begin() + source, adjacency_list_edge_t(source, destination, info));
    else
        edges.emplace_back(source, destination, info);

    // shift following offsets
    for (node_id_t following_offset = 0; following_offset < offsets.size(); following_offset++)
        offsets[following_offset]++;
}

template<typename E>
void
adjacency_list<E>::remove_edge(node_id_t &&source, node_id_t &&destination) {
    edge_id_t edge_idx = edge_index(source, destination);

    edges.erase(edge_idx);

    // shift following offsets
    for (node_id_t following_offset = 0; following_offset < offsets.size(); following_offset++)
        offsets[following_offset]--;
}

// TODO refactor using adjacency_list_builder
template<typename E>
adjacency_list<E>
adjacency_list<E>::inverse(std::vector<node_id_t>& forward_ids, std::vector<node_id_t>& backward_ids) const {
    std::vector<std::vector<edge_id_t> > incoming_edges(node_count());

    for (size_t edge_index = 0; edge_index < edge_count(); edge_index++) {
        const adjacency_list_edge_t<E> &edge = edges[edge_index];
        // insert nodes in inverse order
        incoming_edges[node_count() - edge.destination - 1].push_back(edge_index);
    }

    std::vector<edge_id_t> inv_offsets;
    inv_offsets.reserve(node_count());
    std::vector<adjacency_list_edge_t<E> > inv_edges;
    inv_edges.reserve(edge_count());

    // store new indices for each edge
    backward_ids.clear();
    backward_ids.reserve(edge_count());
    forward_ids.clear();
    forward_ids.resize(edge_count());

    edge_id_t current = 0;
    for (size_t node_index = 0; node_index < node_count(); node_index++) {
        auto incoming = incoming_edges[node_count() - node_index - 1];
        for (edge_id_t edge: incoming) {
            forward_ids[current] = edge;
            backward_ids[edge] = current++;
        }
    }

    for (size_t node_index = 0; node_index < node_count(); node_index++) {
        inv_offsets.push_back(inv_edges.size());

        auto incoming = incoming_edges.back();
        for (edge_id_t edge_id: incoming) {
            const adjacency_list_edge_t<E> &edge = edges[edge_id];
            inv_edges.push_back(edge.invert(backward_ids));
        }

        incoming_edges.pop_back();
    }

    return adjacency_list(node_count(), edge_count(), std::move(inv_offsets), std::move(inv_edges));
}

template<typename E>
bool
adjacency_list<E>::operator==(const adjacency_list<E> &other) {
    if (nodecount != other.node_count || edgecount != other.edge_count)
        return false;

    for (node_id_t i = 0; i < nodecount; i++)
        if (offsets[i] != other.offsets[i]) return false;

    for (edge_id_t i = 0; i < edgecount; i++)
        if (edges[i] != other.edges[i]) return false;

    return true;
}
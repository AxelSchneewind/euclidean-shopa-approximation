#pragma once

#include "node_info_array.h"
#include <span>

template<typename EdgeId, typename IntraEdgeId, typename Info>
std::span<const Info> node_info_array<EdgeId, IntraEdgeId, Info>::node_infos(edge_id_type edge_id) const {
    auto of = offset(edge_id);
    auto ofn = offset_next(edge_id);
    return {node_info_list.begin() + of, node_info_list.begin() + ofn};
}

template<typename EdgeId, typename IntraEdgeId, typename Info>
node_info_array<EdgeId, IntraEdgeId, Info>::index_type node_info_array<EdgeId, IntraEdgeId, Info>::offset_next(edge_id_type edge_id) const {
    assert(edge_id + 1 < offsets.size());
    return offsets[edge_id + 1];
}

template<typename EdgeId, typename IntraEdgeId, typename Info>
void node_info_array<EdgeId, IntraEdgeId, Info>::reset(edge_id_type edge_id) {
    auto of = offset(edge_id);
    auto ofn = offset_next(edge_id);
    std::fill(node_info_list.begin() + of, node_info_list.begin() + ofn, default_value);
}

template<typename EdgeId, typename IntraEdgeId, typename Info>
void node_info_array<EdgeId, IntraEdgeId, Info>::reset(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
    auto of = offset(edge_id);
    node_info_list[of + intra_edge_id] = default_value;
}

template<typename EdgeId, typename IntraEdgeId, typename Info>
Info
node_info_array<EdgeId, IntraEdgeId, Info>::node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) const {
    auto of = offset(edge_id);
    return node_info_list[of + intra_edge_id];
}

template<typename EdgeId, typename IntraEdgeId, typename Info>
std::span<Info> node_info_array<EdgeId, IntraEdgeId, Info>::node_infos(edge_id_type edge_id) {
    auto of = offset(edge_id);
    auto ofn = offset_next(edge_id);
    return {node_info_list.begin() + of, node_info_list.begin() + ofn};
}

template<typename EdgeId, typename IntraEdgeId, typename Info>
Info &node_info_array<EdgeId, IntraEdgeId, Info>::node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
    auto of = offset(edge_id);
    return node_info_list[of + intra_edge_id];
}

template<typename EdgeId, typename IntraEdgeId, typename Info>
node_info_array<EdgeId, IntraEdgeId, Info>::node_info_array(const std::vector<index_type> &offsets, Info default_value)
        : offsets(offsets), default_value(default_value), node_info_list(offsets.back(), default_value) {}

template<typename EdgeId, typename IntraEdgeId, typename Info>
node_info_array<EdgeId, IntraEdgeId, Info>::node_info_array(std::vector<index_type> &&offsets, Info default_value) : offsets(std::move(offsets)), default_value(default_value), node_info_list(offsets.back(), default_value) {}

template<typename EdgeId, typename IntraEdgeId, typename Info>
node_info_array<EdgeId, IntraEdgeId, Info>::index_type node_info_array<EdgeId, IntraEdgeId, Info>::offset(edge_id_type edge_id) const {
    assert(edge_id < offsets.size() - 1);
    return offsets[edge_id];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
void node_info_array<AggregateId, IntraAggregateId, Info>::reset() {
    for (int i = 0; i < node_info_list.size(); ++i) {
        node_info_list[i] = default_value;
    }
}

template<typename EdgeId, typename IntraEdgeId, typename Info>
size_t node_info_array<EdgeId, IntraEdgeId, Info>::edge_count() const { return offsets.back(); }

template<typename EdgeId, typename IntraEdgeId, typename Info>
size_t node_info_array<EdgeId, IntraEdgeId, Info>::node_count(edge_id_type edge_id) const { return offsets[edge_id + 1] - offsets[edge_id]; }

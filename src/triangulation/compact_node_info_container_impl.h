#pragma once

#include "compact_node_info_container.h"

#include <vector>
#include <map>

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
Info const&
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::node_info(edge_id_type edge_id,
                                                                                           intra_edge_id_type intra_edge_id) const {
    return get(edge_id, intra_edge_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
Info const& compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::get(AggregateId agg_id,
                                                                                          IntraAggregateId intra_id) const {
    if (!is_expanded(agg_id)) return default_info;

    std::vector<Info> const &list = aggregate_info_ptr.at(agg_id)->info;
    return list[intra_id];
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
bool
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::is_expanded(AggregateId agg_id) const {
    return aggregate_info_ptr.contains(agg_id) && aggregate_info_ptr.at(agg_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
size_t compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::aggregate_count() const {
    return edge_count;
}


template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
std::span<const Info>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::node_infos(
        edge_id_type edge_id) const {
    return get(edge_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::reset(edge_id_type edge_id) {
    erase(edge_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::reset() {
    aggregate_info_ptr.clear();
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::reset(edge_id_type edge_id,
                                                                                            intra_edge_id_type intra_edge_id) {
    aggregate_info_ptr[edge_id]->info[intra_edge_id] = default_info;
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
std::span<Info>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::node_infos(edge_id_type edge_id) {
    return get(edge_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
Info &
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::node_info(edge_id_type edge_id,
                                                                                           intra_edge_id_type intra_edge_id) {
    if (!is_expanded(edge_id)) {
        auto &&of = _offsets[edge_id];
        auto &&ofn = _offsets[edge_id + 1];
        expand(edge_id, ofn - of);
    }
    return get(edge_id, intra_edge_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::clear() {
    for (int i = 0; i < aggregate_info_ptr.size(); ++i) {
        aggregate_info_ptr[i] = nullptr;
    }
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
AggregateInfo &
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::get_aggregate_info(
        AggregateId agg_id) {
    assert(is_expanded(agg_id));
    return aggregate_info_ptr[agg_id]->aggregate_info;
}
template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
AggregateInfo const&
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::get_aggregate_info(
        AggregateId agg_id) const {
    assert(is_expanded(agg_id));
    return aggregate_info_ptr[agg_id]->aggregate_info;
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
Info &compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::get(AggregateId agg_id,
                                                                                           IntraAggregateId intra_id) {
    assert(is_expanded(agg_id));
    std::vector<Info> &list = aggregate_info_ptr[agg_id]->info;
    return list[intra_id];
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
std::span<Info, std::dynamic_extent>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::get(AggregateId agg_id) {
    assert(is_expanded(agg_id));
    auto &info = aggregate_info_ptr[agg_id];
    std::vector<Info> &list = info.info;
    assert(agg_id == info.aggregate_id);
    assert(!list.empty());
    return {list.begin(), list.end()};
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
std::span<const Info, std::dynamic_extent>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::get(AggregateId agg_id) const {
    assert(is_expanded(agg_id));

    auto &info = aggregate_info_ptr.at(agg_id);
    std::vector<Info> const &list = info->info;
    assert(agg_id == info->aggregate_id);
    assert(!list.empty());
    return {list.begin(), list.end()};
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::put_aggregate_info(AggregateId agg_id,
                                                                                                    AggregateInfo info) {
    assert(is_expanded(agg_id));

    auto &&agg = *aggregate_info_ptr[agg_id];
    agg.aggregate_info = info;
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::put(AggregateId agg_id,
                                                                                          IntraAggregateId intra_id,
                                                                                          Info info) {
    assert (is_expanded(agg_id));
    auto &&agg = *aggregate_info_ptr[agg_id];
    agg.info[intra_id] = info;
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::erase(AggregateId agg_id) {
    if (!is_expanded(agg_id)) [[unlikely]] return;

    _node_count -= aggregate_info_ptr[agg_id]->info.size();
    _edge_count--;
    aggregate_info_ptr[agg_id].reset();

    assert(!is_expanded(agg_id));
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
constexpr compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::aggregate::aggregate(
        AggregateId aggregate_id, AggregateInfo aggregate_info, std::vector<Info> info) : aggregate_id(aggregate_id),
                                                                                          aggregate_info(
                                                                                                  aggregate_info),
                                                                                          info(info) {}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
size_t compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::aggregate_size(
        AggregateId agg_id) const {
    return aggregate_info_ptr.contains(agg_id) ? aggregate_info_ptr.at(agg_id)->info.size() : 0;
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::aggregate::aggregate(
        compact_node_info_container::aggregate &&other) : aggregate_id(other.aggregate_id),
                                                          aggregate_info(other.aggregate_info),
                                                          info(std::move(other.info)) {}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::compact_node_info_container(
        std::vector<size_t> &&offsets, AggregateInfo default_aggregate_info, Info default_info)
        : default_aggregate_info(default_aggregate_info), _offsets(std::move(offsets)), default_info(default_info),
          _edge_count(0), _node_count(0), default_span{default_info} {}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::compact_node_info_container(
        const std::vector<size_t> &offsets, AggregateInfo default_aggregate_info, Info default_info)
        : default_aggregate_info(default_aggregate_info), _offsets(offsets), default_info(default_info),
          default_span{default_info} {}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::expand(AggregateId agg_id,
                                                                                             IntraAggregateId count) {
    if (is_expanded(agg_id)) [[unlikely]] return;
    assert(count > 0);

    aggregate_info_ptr[agg_id] = std::make_unique<aggregate>(agg_id, default_aggregate_info,
                                                             std::vector(count, default_info));
    _edge_count++;
    _node_count += count;

    assert(aggregate_size(agg_id) > 0);
    assert(is_expanded(agg_id));
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
size_t
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::edge_count() const { return _edge_count; }

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
size_t
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::node_count(
        edge_id_type edge_id) const { return is_expanded(edge_id) ? get(edge_id).size() : 0; }

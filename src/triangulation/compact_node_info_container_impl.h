#pragma once

#include "compact_node_info_container.h"

#include <vector>
#include <map>
#include "node_info_container.h"

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
Info
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::node_info(edge_id_type edge_id,
                                                                                           intra_edge_id_type intra_edge_id) const {
    return get(edge_id, intra_edge_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
Info compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::get(AggregateId agg_id,
                                                                                          IntraAggregateId intra_id) const {
    if (!is_expanded(agg_id)) return default_info;
    // assert(aggregate_info_index.contains(agg_id));
    // info_index_type index = aggregate_info_index[agg_id];
    // assert(index < aggregate_count());
    // assert(intra_id < aggregate_size(agg_id));
    // std::vector<Info> const &list = aggregate_info[index].info;
    std::vector<Info> const &list = aggregate_info_ptr.at(agg_id)->info;
    return list[intra_id];
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
bool
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::is_expanded(AggregateId agg_id) const {
    // assert(!aggregate_info_index.contains(agg_id) || aggregate_info.at(aggregate_info_index.at(agg_id)).aggregate_id == agg_id);
    // return aggregate_info_index.contains(agg_id);
    return aggregate_info_ptr.contains(agg_id) && aggregate_info_ptr.at(agg_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
size_t compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::aggregate_count() const {
    //return aggregate_info_index.size();

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
        auto &&of = offsets[edge_id];
        auto &&ofn = offsets[edge_id + 1];
        expand(edge_id, ofn - of);
    }
    return get(edge_id, intra_edge_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::clear() {
    // aggregate_info.clear();
    // aggregate_info_index.clear();
    for (int i = 0; i < aggregate_info_ptr.size(); ++i) {
        aggregate_info_ptr[i] = nullptr;
    }
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
AggregateInfo
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::get_aggregate_info(
        AggregateId agg_id) {
    assert(is_expanded(agg_id));
    if (is_expanded(agg_id)) {
        // assert(aggregate_info_index.contains(agg_id));
        // info_index_type index = aggregate_info_index[agg_id];
        // assert(index < aggregate_count());
        // return aggregate_info[index].aggregate_info;
        return aggregate_info_ptr[agg_id]->aggregate_info;
    } else {
        return default_aggregate_info;
    }
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
Info &compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::get(AggregateId agg_id,
                                                                                           IntraAggregateId intra_id) {
    assert(is_expanded(agg_id));
    // assert(aggregate_info_index.contains(agg_id));
    // info_index_type index = aggregate_info_index[agg_id];
    // assert(index < aggregate_count());
    // assert(intra_id < aggregate_size(agg_id));
    // std::vector<Info> const &list = aggregate_info[index].info;
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

    // info_index_type index = aggregate_info_index[agg_id];
    // assert(index < aggregate_count());
    // aggregate_info[index].aggregate_info = info;

    // assert(aggregate_info[aggregate_info_index[agg_id]].aggregate_id == agg_id);

    auto &&agg = *aggregate_info_ptr[agg_id];
    agg.aggregate_info = info;
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::put(AggregateId agg_id,
                                                                                          IntraAggregateId intra_id,
                                                                                          Info info) {
    assert (is_expanded(agg_id));

    // info_index_type index = aggregate_info_index[agg_id];
    // assert(index < aggregate_count());

    // std::vector<Info> &list = aggregate_info[index].info;

    // assert(intra_id < aggregate_size(agg_id));
    // list[intra_id] = info;

    // assert(aggregate_info[aggregate_info_index[agg_id]].aggregate_id == agg_id);
    // assert(!aggregate_info_index.contains(agg_id) || aggregate_info.at(aggregate_info_index.at(agg_id)).aggregate_id == agg_id);

    auto &&agg = *aggregate_info_ptr[agg_id];
    agg.info[intra_id] = info;
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::erase(AggregateId agg_id) {
    assert(is_expanded(agg_id));
    // info_index_type index = aggregate_info_index[agg_id];

    // // swap last element in vector here and update the index of its aggregate
    // AggregateId other_agg_id = aggregate_info.back().aggregate_id;

    // if (agg_id == other_agg_id) {
    //     aggregate_info.pop_back();
    // } else {
    //     assert(aggregate_info_index.contains(other_agg_id));
    //     aggregate_info[index] = std::move(aggregate_info.back());
    //     aggregate_info_index[other_agg_id] = index;
    // }

    // // remove index for this aggregate
    // aggregate_info_index.erase(agg_id);
    // // remove aggregate info
    // aggregate_info.pop_back();

    _node_count -= aggregate_info_ptr[agg_id]->info.size();
    _edge_count--;
    aggregate_info_ptr[agg_id] = nullptr;

    // assert(agg_id == other_agg_id || aggregate_info_index.contains(other_agg_id));
    // assert(agg_id == other_agg_id || aggregate_info.at(aggregate_info_index.at(other_agg_id)).aggregate_id == other_agg_id);
    // assert(!aggregate_info_index.contains(agg_id));
    // assert(!is_expanded(agg_id));
    // assert(!aggregate_info_index.contains(agg_id) || aggregate_info.at(aggregate_info_index.at(agg_id)).aggregate_id == agg_id);
    // assert(!aggregate_info_index.contains(other_agg_id) || aggregate_info.at(aggregate_info_index.at(other_agg_id)).aggregate_id == other_agg_id);
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
    //return aggregate_info[aggregate_info_index.at(agg_id)].info.size();
    return aggregate_info_ptr.contains(agg_id) ? aggregate_info_ptr.at(agg_id)->info.size() : 0;
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::aggregate::aggregate(
        compact_node_info_container::aggregate &&other) : aggregate_id(other.aggregate_id),
                                                          aggregate_info(other.aggregate_info),
                                                          info(std::move(other.info)) {}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::compact_node_info_container(
        std::vector<unsigned int> &&offsets, AggregateInfo default_aggregate_info, Info default_info)
        : default_aggregate_info(default_aggregate_info)
        , offsets(std::move(offsets))
        , default_info(default_info)
        , _edge_count(0)
        , _node_count(0)
        , default_span{default_info} {}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::compact_node_info_container(
        const std::vector<unsigned int> &offsets, AggregateInfo default_aggregate_info, Info default_info)
        : default_aggregate_info(default_aggregate_info), offsets(offsets), default_info(default_info),
          default_span{default_info} {}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
void compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::expand(AggregateId agg_id,
                                                                                             IntraAggregateId count) {
    assert(!is_expanded(agg_id));
    assert(count > 0);
    // assert(aggregate_count() + 1 < (size_t)std::numeric_limits<info_index_type>::max);
    // assert(!aggregate_info_index.contains(agg_id));

    //aggregate_info_index[agg_id] = aggregate_info.size();
    //aggregate_info.emplace_back(agg_id, default_aggregate_info, std::vector(count, default_info));
    aggregate_info_ptr[agg_id] = std::make_unique<aggregate>(agg_id, default_aggregate_info,
                                                             std::vector(count, default_info));
    _edge_count++;
    _node_count += count;

    assert(aggregate_size(agg_id) > 0);
    // assert(aggregate_info[aggregate_info_index[agg_id]].aggregate_id == agg_id);
}

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
size_t
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::edge_count() const { return _edge_count; }

template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
size_t
compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>::node_count(
        edge_id_type edge_id) const { return is_expanded(edge_id) ? get(edge_id).size() : 0; }

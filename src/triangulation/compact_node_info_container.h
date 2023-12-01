#pragma once


#include <vector>
#include <map>
#include "node_info_container.h"


template<typename AggregateId, typename IntraAggregateId, typename AggregateInfo, typename Info>
class compact_node_info_container
        : public node_info_container<AggregateId, IntraAggregateId, Info, compact_node_info_container<AggregateId, IntraAggregateId, AggregateInfo, Info>>{
public:
    using edge_id_type = AggregateId;
    using intra_edge_id_type = IntraAggregateId;
    using info_type = Info;
private:
    using info_index_type = unsigned int;

    struct aggregate {
        AggregateId aggregate_id;
        AggregateInfo aggregate_info;
        std::vector<Info> info;

        constexpr aggregate(AggregateId aggregate_id, AggregateInfo aggregate_info, std::vector<Info> info) : aggregate_id(aggregate_id), aggregate_info(aggregate_info), info(info) {}

        aggregate(aggregate&& other) : aggregate_id(other.aggregate_id), aggregate_info(other.aggregate_info), info(std::move(other.info)) {}

        aggregate& operator=(aggregate&& other) = default;
    };

    // std::vector<aggregate> aggregate_info;
    // std::unordered_map<AggregateId, info_index_type> aggregate_info_index;
    std::unordered_map<AggregateId, std::unique_ptr<aggregate>> aggregate_info_ptr;
    std::vector<unsigned int> offsets;

    size_t edge_count;

    AggregateInfo default_aggregate_info;
    Info default_info;
    std::array<Info, 1> default_span;

public:
    compact_node_info_container(std::vector<unsigned int> const& offsets, AggregateInfo default_aggregate_info = none_value<AggregateInfo>, Info default_info = none_value<Info>)
            : default_aggregate_info(default_aggregate_info)
            , offsets(offsets)
            , default_info(default_info), default_span{default_info} {};

    compact_node_info_container(std::vector<unsigned int>&& offsets, AggregateInfo default_aggregate_info = none_value<AggregateInfo>,
                                Info default_info = none_value<Info>)
            : default_aggregate_info(default_aggregate_info)
            , offsets(std::move(offsets))
            , default_info(default_info), default_span{default_info} {};

    size_t aggregate_size(AggregateId agg_id) const {
        //return aggregate_info[aggregate_info_index.at(agg_id)].info.size();
        return aggregate_info_ptr.contains(agg_id) ? aggregate_info_ptr.at(agg_id)->info.size() : 0;
    }

    size_t aggregate_count() const {
        //return aggregate_info_index.size();

        return edge_count;
    }

private:
    // FIXME
    void expand(AggregateId agg_id, IntraAggregateId count) {
        assert(!is_expanded(agg_id));
        assert(count > 0);
        // assert(aggregate_count() + 1 < (size_t)std::numeric_limits<info_index_type>::max);
        // assert(!aggregate_info_index.contains(agg_id));

        //aggregate_info_index[agg_id] = aggregate_info.size();
        //aggregate_info.emplace_back(agg_id, default_aggregate_info, std::vector(count, default_info));
        aggregate_info_ptr[agg_id] = std::make_unique<aggregate>(agg_id, default_aggregate_info, std::vector(count, default_info));
        edge_count++;

        assert(aggregate_size(agg_id) > 0);
        // assert(aggregate_info[aggregate_info_index[agg_id]].aggregate_id == agg_id);
    }

    // FIXME
    void erase(AggregateId agg_id) {
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

        aggregate_info_ptr[agg_id] = nullptr;
        edge_count--;

        // assert(agg_id == other_agg_id || aggregate_info_index.contains(other_agg_id));
        // assert(agg_id == other_agg_id || aggregate_info.at(aggregate_info_index.at(other_agg_id)).aggregate_id == other_agg_id);
        // assert(!aggregate_info_index.contains(agg_id));
        // assert(!is_expanded(agg_id));
        // assert(!aggregate_info_index.contains(agg_id) || aggregate_info.at(aggregate_info_index.at(agg_id)).aggregate_id == agg_id);
        // assert(!aggregate_info_index.contains(other_agg_id) || aggregate_info.at(aggregate_info_index.at(other_agg_id)).aggregate_id == other_agg_id);
    }

    bool is_expanded(AggregateId agg_id) const {
        // assert(!aggregate_info_index.contains(agg_id) || aggregate_info.at(aggregate_info_index.at(agg_id)).aggregate_id == agg_id);
        // return aggregate_info_index.contains(agg_id);
        return aggregate_info_ptr.contains(agg_id) && aggregate_info_ptr.at(agg_id);
    }

    void put(AggregateId agg_id, IntraAggregateId intra_id, Info info) {
        assert (is_expanded(agg_id));

        // info_index_type index = aggregate_info_index[agg_id];
        // assert(index < aggregate_count());

        // std::vector<Info> &list = aggregate_info[index].info;

        // assert(intra_id < aggregate_size(agg_id));
        // list[intra_id] = info;

        // assert(aggregate_info[aggregate_info_index[agg_id]].aggregate_id == agg_id);
        // assert(!aggregate_info_index.contains(agg_id) || aggregate_info.at(aggregate_info_index.at(agg_id)).aggregate_id == agg_id);

        auto&& agg = *aggregate_info_ptr[agg_id];
        agg.info[intra_id] = info;
    }

    void put_aggregate_info(AggregateId agg_id, AggregateInfo info) {
        assert(is_expanded(agg_id));

        // info_index_type index = aggregate_info_index[agg_id];
        // assert(index < aggregate_count());
        // aggregate_info[index].aggregate_info = info;

        // assert(aggregate_info[aggregate_info_index[agg_id]].aggregate_id == agg_id);

        auto&& agg = *aggregate_info_ptr[agg_id];
        agg.aggregate_info = info;
    }


    std::span<Info, std::dynamic_extent> get(AggregateId agg_id) {
        assert(is_expanded(agg_id));
        if (is_expanded(agg_id)) {
            // assert(aggregate_info_index.contains(agg_id));
            // info_index_type index = aggregate_info_index[agg_id];
            // assert(index < aggregate_count());
            // auto& info = aggregate_info[index];
            auto& info = aggregate_info_ptr[agg_id];
            std::vector<Info> &list = info.info;
            assert(agg_id == info.aggregate_id);
            assert(!list.empty());
            return {list.begin(), list.end()};
        }
        default_span[0] = default_info;

        return {default_span};
    }

    Info get(AggregateId agg_id, IntraAggregateId intra_id) const {
        if(!is_expanded(agg_id)) return default_info;
        // assert(aggregate_info_index.contains(agg_id));
        // info_index_type index = aggregate_info_index[agg_id];
        // assert(index < aggregate_count());
        // assert(intra_id < aggregate_size(agg_id));
        // std::vector<Info> const &list = aggregate_info[index].info;
        std::vector<Info> const &list = aggregate_info_ptr.at(agg_id)->info;
        return list[intra_id];
    }
    Info& get(AggregateId agg_id, IntraAggregateId intra_id) {
        assert(is_expanded(agg_id));
            // assert(aggregate_info_index.contains(agg_id));
            // info_index_type index = aggregate_info_index[agg_id];
            // assert(index < aggregate_count());
            // assert(intra_id < aggregate_size(agg_id));
            // std::vector<Info> const &list = aggregate_info[index].info;
            std::vector<Info>& list = aggregate_info_ptr[agg_id]->info;
            return list[intra_id];
    }

    AggregateInfo get_aggregate_info(AggregateId agg_id) {
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

    void clear() {
        // aggregate_info.clear();
        // aggregate_info_index.clear();
        for (int i = 0; i < aggregate_info_ptr.size(); ++i) {
            aggregate_info_ptr[i] = nullptr;
        }
    }

public:

    info_type& node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
        if (!is_expanded(edge_id)) {
            auto&& of = offsets[edge_id];
            auto&& ofn = offsets[edge_id + 1];
            expand(edge_id, ofn - of);
        }
        return get(edge_id, intra_edge_id);
    }

    info_type node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) const {
        return get(edge_id, intra_edge_id);
    }

    std::span<info_type> node_infos(edge_id_type edge_id) {
        return get(edge_id);
    }

    std::span<const info_type> node_infos(edge_id_type edge_id) const {
        return get(edge_id);
    }

    void reset(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
        aggregate_info_ptr[edge_id]->info[intra_edge_id] = default_info;
    }

    void reset(edge_id_type edge_id) {
        erase(edge_id);
    }
};
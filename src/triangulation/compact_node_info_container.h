#pragma once

#include "node_info_container.h"

#include <vector>
#include <map>

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

        constexpr aggregate(AggregateId aggregate_id, AggregateInfo aggregate_info, std::vector<Info> info);

        aggregate(aggregate&& other);

        aggregate& operator=(aggregate&& other) = default;
    };

    // std::vector<aggregate> aggregate_info;
    // std::unordered_map<AggregateId, info_index_type> aggregate_info_index;
    std::unordered_map<AggregateId, std::unique_ptr<aggregate>> aggregate_info_ptr;
    std::vector<size_t> _offsets;

    size_t _edge_count;
    size_t _node_count;

    AggregateInfo default_aggregate_info;
    Info default_info;
    std::array<Info, 1> default_span;

public:
    compact_node_info_container(std::vector<size_t> const& offsets, AggregateInfo default_aggregate_info = none_value<AggregateInfo>, Info default_info = none_value<Info>);;

    compact_node_info_container(std::vector<size_t>&& offsets, AggregateInfo default_aggregate_info = none_value<AggregateInfo>,
                                Info default_info = none_value<Info>);;

    size_t aggregate_size(AggregateId agg_id) const;

    size_t aggregate_count() const;

private:
    void put(AggregateId agg_id, IntraAggregateId intra_id, Info info);

    void put_aggregate_info(AggregateId agg_id, AggregateInfo info);


    std::span<Info, std::dynamic_extent> get(AggregateId agg_id);
    std::span<const Info, std::dynamic_extent> get(AggregateId agg_id) const;

    [[gnu::hot]]
    Info const& get(AggregateId agg_id, IntraAggregateId intra_id) const;
    [[gnu::hot]]
    Info& get(AggregateId agg_id, IntraAggregateId intra_id);

    AggregateInfo get_aggregate_info(AggregateId agg_id);

    void clear();

public:
    [[gnu::hot]]
    void expand(AggregateId agg_id, IntraAggregateId count);

    void erase(AggregateId agg_id);

    [[gnu::hot]]
    bool is_expanded(AggregateId agg_id) const;


    info_type& node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id);

    info_type const& node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) const;

    std::span<info_type> node_infos(edge_id_type edge_id);

    std::span<const info_type> node_infos(edge_id_type edge_id) const;

    void reset(edge_id_type edge_id, intra_edge_id_type intra_edge_id);

    void reset(edge_id_type edge_id);

    void reset();

    size_t node_count(edge_id_type edge_id) const;
    size_t edge_count() const;
};

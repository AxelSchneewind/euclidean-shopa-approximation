#pragma once

#include <cstddef>
#include <vector>


template <typename AggregateId, typename IntraAggregateId, typename Info>
class fast_map {
private:
    using aggregate_id_type = AggregateId;
    using intra_aggregate_id_type = IntraAggregateId;
    using index = int;
    using entry_type = Info;

    std::vector<index> _edge_index;
    std::vector<entry_type> _entries;

    entry_type _default_value;


    void clear();

public:
    using edge_id_type = AggregateId;
    using intra_edge_id_type = IntraAggregateId;
    using info_type = Info;

    fast_map (std::size_t edge_count, Info default_value);

    bool contains(aggregate_id_type id) const;

    void append(aggregate_id_type id, intra_aggregate_id_type count);

    info_type& at(edge_id_type edge_id, intra_edge_id_type intra_edge_id);

    info_type const& at(edge_id_type edge_id, intra_edge_id_type intra_edge_id) const;

    template<typename Pair> requires requires(Pair t) {t.edge; t.steiner_index; }
    info_type& operator[](Pair&& index);

    template<typename Pair> requires requires(Pair t) {t.edge; t.steiner_index; }
    info_type const& operator[](Pair&& index) const;

    void reset();

    std::size_t edge_count() const;

    std::size_t size() const;

    void clear();
};

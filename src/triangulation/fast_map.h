#pragma once

#include <cstddef>
#include <vector>


/**
 * an offset array based map for two-dimensional keys.
 * @tparam AggregateId first key
 * @tparam IntraAggregateId second key
 * @tparam Info elements to store
 */
template <typename AggregateId, typename IntraAggregateId, typename Index, typename Info>
class fast_map {
private:
    using aggregate_id_type = AggregateId;
    using intra_aggregate_id_type = IntraAggregateId;
    using index_type = Index;
    using entry_type = Info;

    std::vector<index_type> _aggregate_index;
    std::vector<entry_type> _entries;

public:
    using info_type = Info;

    fast_map(size_t aggregate_count);

    bool contains(aggregate_id_type id) const;

    void append(aggregate_id_type id, intra_aggregate_id_type count);
    void append(aggregate_id_type id, intra_aggregate_id_type count, entry_type values);

    info_type& at(aggregate_id_type edge_id, intra_aggregate_id_type intra_edge_id);

    info_type const& at(aggregate_id_type edge_id, intra_aggregate_id_type intra_edge_id) const;

    template<typename Pair> requires requires(Pair t) {t.first(); t.second(); }
    info_type& operator[](Pair&& index);

    template<typename Pair> requires requires(Pair t) {t.first(); t.second(); }
    info_type const& operator[](Pair&& index) const;

    void reset();

    std::size_t edge_count() const;

    std::size_t size() const;

    void clear();

    void shrink_to_fit();
};

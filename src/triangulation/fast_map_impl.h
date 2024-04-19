#pragma once

#include "fast_map.h"
#include "../util/contract.h"

#include <algorithm>
#include <cstddef>
#include <vector>

template<typename AggregateId, typename IntraAggregateId, typename Info>
std::size_t fast_map<AggregateId, IntraAggregateId, Info>::edge_count() const { return _aggregate_index.size(); }

template<typename AggregateId, typename IntraAggregateId, typename Info>
void fast_map<AggregateId, IntraAggregateId, Info>::reset() { clear(); }

template<typename AggregateId, typename IntraAggregateId, typename Info>
Info const &
fast_map<AggregateId, IntraAggregateId, Info>::at(aggregate_id_type aggregate_id, intra_aggregate_id_type intra_aggregate_id) const {
    assert(contains(aggregate_id));
    return _entries[_aggregate_index[aggregate_id] + intra_aggregate_id];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
template<typename Pair> requires requires (Pair t) { t.first(); t.second(); }
typename fast_map<AggregateId, IntraAggregateId, Info>::info_type& fast_map<AggregateId, IntraAggregateId, Info>::
operator[](Pair&& index) {
    assert(contains(index.first()));
    return _entries[_aggregate_index[index.first()] + index.second()];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
template<typename Pair> requires requires (Pair t) { t.first(); t.second(); }
typename fast_map<AggregateId, IntraAggregateId, Info>::info_type const& fast_map<AggregateId, IntraAggregateId, Info>::
operator[](Pair&& index) const {
    assert(contains(index.first()));
    return _entries[_aggregate_index[index.first()] + index.second()];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
Info &
fast_map<AggregateId, IntraAggregateId, Info>::at(aggregate_id_type aggregate_id, intra_aggregate_id_type intra_aggregate_id) {
    assert(contains(aggregate_id));
    return _entries[_aggregate_index[aggregate_id] + intra_aggregate_id];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
void fast_map<AggregateId, IntraAggregateId, Info>::append(aggregate_id_type id, intra_aggregate_id_type count) {
    assert(id >= 0 && static_cast<size_t>(id) < _aggregate_index.size());

    _aggregate_index[id] = _entries.size();
    _entries.resize(_entries.size() + count);

    assert(contains(id));
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
void fast_map<AggregateId, IntraAggregateId, Info>::append(aggregate_id_type id, intra_aggregate_id_type count, entry_type values) {
    assert(id >= 0 && static_cast<size_t>(id) < _aggregate_index.size());

    _aggregate_index[id] = _entries.size();
    _entries.resize(_entries.size() + count, values);

    assert(contains(id));
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
bool fast_map<AggregateId, IntraAggregateId, Info>::contains(aggregate_id_type id) const {
    assert(id >= 0 && static_cast<size_t>(id) < _aggregate_index.size());
    return -1 != _aggregate_index[id];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
fast_map<AggregateId, IntraAggregateId, Info>::fast_map(std::size_t aggregate_count)
        : _aggregate_index(aggregate_count, -1)
{}

template<typename AggregateId, typename IntraAggregateId, typename Info>
void fast_map<AggregateId, IntraAggregateId, Info>::clear() {
    std::fill(_aggregate_index.begin(), _aggregate_index.end(), -1);
    _entries.clear();
    _entries.shrink_to_fit();
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
std::size_t fast_map<AggregateId, IntraAggregateId, Info>::size() const { return _entries.size(); }

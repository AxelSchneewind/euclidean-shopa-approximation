#include "fast_map.h"
#include "../util/contract.h"

#include <algorithm>
#include <cstddef>
#include <vector>

template<typename AggregateId, typename IntraAggregateId, typename Info>
std::size_t fast_map<AggregateId, IntraAggregateId, Info>::edge_count() const { return _edge_index.size(); }

template<typename AggregateId, typename IntraAggregateId, typename Info>
void fast_map<AggregateId, IntraAggregateId, Info>::reset() { clear(); }

template<typename AggregateId, typename IntraAggregateId, typename Info>
Info const &
fast_map<AggregateId, IntraAggregateId, Info>::at(edge_id_type edge_id, intra_edge_id_type intra_edge_id) const {
    if (contains(edge_id)) { [[likely]]
        return _entries[_edge_index[edge_id] + intra_edge_id];
    } else {
        return _default_value;
    }
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
template<typename Pair> requires requires (Pair t) { t.edge; t.steiner_index; }
typename fast_map<AggregateId, IntraAggregateId, Info>::info_type& fast_map<AggregateId, IntraAggregateId, Info>::
operator[](Pair&& index) {
    return _entries[_edge_index[index.edge] + index.steiner_index];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
template<typename Pair> requires requires (Pair t) { t.edge; t.steiner_index; }
typename fast_map<AggregateId, IntraAggregateId, Info>::info_type const& fast_map<AggregateId, IntraAggregateId, Info>::
operator[](Pair&& index) const {
    return _entries[_edge_index[index.edge] + index.steiner_index];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
Info &
fast_map<AggregateId, IntraAggregateId, Info>::at(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
    assert(contains(edge_id));
    return _entries[_edge_index[edge_id] + intra_edge_id];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
void fast_map<AggregateId, IntraAggregateId, Info>::append(aggregate_id_type id, intra_aggregate_id_type count) {
    assert(id >= 0 && static_cast<size_t>(id) < _edge_index.size());

    _edge_index[id] = _entries.size();
    _entries.resize(_entries.size() + count, _default_value);

    assert(contains(id));
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
bool fast_map<AggregateId, IntraAggregateId, Info>::contains(aggregate_id_type id) const {
    assert(id >= 0 && static_cast<size_t>(id) < _edge_index.size());
    return -1 != _edge_index[id];
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
fast_map<AggregateId, IntraAggregateId, Info>::fast_map(std::size_t edge_count, Info default_value)
        : _edge_index(edge_count, -1)
        , _default_value{default_value}
{}

template<typename AggregateId, typename IntraAggregateId, typename Info>
void fast_map<AggregateId, IntraAggregateId, Info>::clear() {
    std::fill(_edge_index.begin(), _edge_index.end(), -1);
    _entries.clear();
    _entries.shrink_to_fit();
}

template<typename AggregateId, typename IntraAggregateId, typename Info>
std::size_t fast_map<AggregateId, IntraAggregateId, Info>::size() const { return _entries.size(); }

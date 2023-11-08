#pragma once

#include "base_types.h"

#include <limits>
#include <vector>
#include <complex>

edge_t
edge_t::invert(const std::vector<edge_id_t> &) const { return edge_t{cost}; }

ch_edge_t
ch_edge_t::invert(const std::vector<edge_id_t> &__new_index) const {
    return ch_edge_t{cost,
                     edgeB ? __new_index[edgeB] : none_value<edge_id_t>(),
                     edgeA
                     ? __new_index[edgeA]
                     : none_value<edge_id_t>()};
}


#pragma once

#include "base_types.h"

#include <limits>
#include <vector>
#include <complex>

edge_t
edge_t::invert (const std::vector<edge_id_t> &) const
{ return edge_t{cost}; }

ch_edge_t
ch_edge_t::invert (const std::vector<edge_id_t> &__new_index) const
{
  return ch_edge_t{cost,
		   edgeB != NO_EDGE_ID ? __new_index[edgeB] : NO_EDGE_ID,
		   edgeA != NO_NODE_ID
		   ? __new_index[edgeA]
		   : NO_NODE_ID};
}


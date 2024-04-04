#pragma once
#include <concepts>

template <sized_random_access_range NodeRange, sized_random_access_range PropertyRange, typename Coords>
requires requires (typename PropertyRange::value_type t) {
    { t.is_in_box } -> std::convertible_to<bool>;
}
void compute_is_in_box(NodeRange const& nodes, PropertyRange& properties, Coords&& bot_left, Coords&& top_right) {
    size_t node_count = std::ranges::size(nodes);
    assert(std::ranges::size(properties) == node_count);

    for (size_t i = 0; i < node_count; ++i) {
        properties[i].is_in_box = is_in_rectangle(nodes[i].coordinates, bot_left, top_right);
    }
}

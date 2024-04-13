#pragma once

#include "../graph/adjacency_list.h"
#include "polyhedron.h"

#include <limits>
#include <utility>
#include <vector>
#include <cstdint>
#include <cmath>


class subdivision {
public:
    using steiner_index_type = int;

    struct subdivision_edge_info {
        // (1+ epsilon * sin(alpha)) for both sides of this edge
        double base_first;
        double base_second;

        // r(v), relative to this edge
        float r_first;

        // relative position of the points with the highest distance to other edges
        float mid_position;

        // r(w)
        float r_second;

        // number of steiner points on this edge (counting the source and if present, middle nodes), >= 2
        steiner_index_type node_count;

        steiner_index_type mid_index;

        bool operator==(const subdivision_edge_info &other) const = default;
    };

    static constexpr size_t max_steiner_count_per_edge = std::numeric_limits<steiner_index_type>::max();
private:

    // here, some lower bounds can be imposed to prevent numerical issues
    static constexpr long double min_r_value = 0x1p-10;

    std::vector<subdivision_edge_info> _edges;

    size_t edges_capped{0};

    subdivision(std::vector<subdivision_edge_info> &&edges) : _edges{std::move(edges)} {};


public:
    static constexpr std::size_t SIZE_PER_NODE = 0;
    static constexpr std::size_t SIZE_PER_EDGE = sizeof(subdivision_edge_info);

    ~subdivision() = default;

    subdivision(subdivision &&other) noexcept = default;

    [[using gnu : hot, pure, always_inline]]
    coordinate_t node_coordinates(edge_id_t edge, steiner_index_type steiner_index, coordinate_t const &c1,
                                  coordinate_t const &c2) const;

    [[using gnu : hot, pure, always_inline]]
    double relative_position(edge_id_t edge, steiner_index_type steiner_index) const;

    [[using gnu : hot, pure, always_inline]]
    double relative_position_mid(edge_id_t edge) const;

    [[using gnu : hot, pure, always_inline]]
    double relative_position_steiner(edge_id_t edge, steiner_index_type steiner_index) const;

    [[using gnu : hot, pure, always_inline]]
    steiner_index_type index(edge_id_t edge, double relative) const;

    [[using gnu : hot, pure, always_inline]]
    subdivision_edge_info &edge(edge_id_t edge);

    [[using gnu : hot, pure, always_inline]]
    subdivision_edge_info const &edge(edge_id_t edge) const;

    // TODO move somewhere else
    [[gnu::cold]]
    std::vector<std::size_t> offsets() const;

    [[gnu::cold]]
    static subdivision make_subdivision_info(
            adjacency_list<int> const&triangulation,
            std::vector<node_t> const&nodes,
            polyhedron<adjacency_list<int>, 3> const&polyhedron,
            std::vector<double> const&r_values,
            double epsilon);
};

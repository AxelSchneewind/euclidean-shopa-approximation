#pragma once

#include "../graph/adjacency_list.h"
#include "polyhedron.h"

#include <cassert>
#include <limits>
#include <utility>
#include <vector>
#include <cstdint>
#include <cmath>


class subdivision {
public:
    using steiner_index_type = int;

private:
    static constexpr steiner_index_type max_steiner_count_per_edge = std::numeric_limits<unsigned short>::max();// std::numeric_limits<steiner_index_type>::max();

    // here, some lower bounds can be imposed to prevent numerical issues
    static constexpr long double min_base = 1.0 + std::numeric_limits<double>::epsilon();
    static constexpr double min_r_value = 0x1.0p-9;

public:
    static constexpr long double min_angle = 0x1.p-48;

    struct subdivision_edge_info {
        // r(v), relative to this edge
        float r_first;

        // relative position of the points with the highest distance to other edges
        float mid_position;

        // r(w)
        float r_second;

        // number of steiner points on this edge (counting the source and if present, middle nodes), >= 2
        steiner_index_type node_count;

        // which classes of edges this one belongs to (first and second half)
        double base_first;
        double base_second;

        steiner_index_type mid_index;

        bool operator==(const subdivision_edge_info &other) const = default;
    };

    std::vector<subdivision_edge_info> edges;

    static constexpr std::size_t SIZE_PER_NODE = 0;
    static constexpr std::size_t SIZE_PER_EDGE = sizeof(subdivision_edge_info);

    ~subdivision() = default;

    subdivision(subdivision &&other) noexcept = default;

    subdivision(std::vector<subdivision_edge_info> &&edges) : edges{std::move(edges)} {};

    [[gnu::hot]]
    [[gnu::pure]]
    [[gnu::always_inline]]
    [[deprecated("use relative_position() functions")]]
    coordinate_t node_coordinates(edge_id_t edge, steiner_index_type steiner_index, coordinate_t const& c1, coordinate_t const& c2) const;

    double relative_position(edge_id_t edge, steiner_index_type steiner_index) const;
    double relative_position_mid(edge_id_t edge) const;
    double relative_position_steiner(edge_id_t edge, steiner_index_type steiner_index) const;
    steiner_index_type index(edge_id_t edge, double relative) const;

    subdivision_edge_info& edge(edge_id_t edge);
    subdivision_edge_info const& edge(edge_id_t edge) const;

    // TODO move somewhere else
    std::vector<std::size_t> offsets() const;

    static std::vector<subdivision_edge_info> make_subdivision_info(
            const adjacency_list<int> &triangulation,
            const std::vector<node_t> &nodes,
            const polyhedron<adjacency_list<int>, 3> &polyhedron,
            const std::vector<double> &r_values,
            double epsilon);
};

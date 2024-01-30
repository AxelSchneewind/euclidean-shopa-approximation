#pragma once

#include "../graph/adjacency_list.h"
#include "polyhedron.h"

#include <limits>
#include <vector>
#include <float.h>
#include <cstdint>
#include <cmath>
#include <cstdint>


class subdivision {
private:
    using steiner_index_type = unsigned int;
    static constexpr steiner_index_type max_steiner_count_per_edge = std::numeric_limits<steiner_index_type>::max() / 2;

public:
    static constexpr double min_angle = 0x1.p-24;

    struct subdivision_edge_info {
        // relative position of the points with the highest distance to other edges
        float mid_position;

        // r(v), relative to this edge
        float r_first;
        float r_second;

        // number of steiner points on this edge (counting the source and if present, middle nodes), >= 2
        unsigned int node_count;

        // which classes of edges this one belongs to (first and second half)
        double base_first;
        double base_second;

        unsigned int mid_index;

        bool operator==(const subdivision_edge_info &other) const = default;
    };

    std::vector<subdivision_edge_info> edges;

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(subdivision_edge_info);

    ~subdivision() = default;

    subdivision(subdivision &&other) noexcept = default;

    subdivision(std::vector<subdivision_edge_info> &&edges) : edges{std::move(edges)} {};

    coordinate_t node_coordinates(edge_id_t edge, steiner_index_type steiner_index, coordinate_t const& c1, coordinate_t const& c2) const;

    subdivision_edge_info& edge(edge_id_t edge);
    subdivision_edge_info const& edge(edge_id_t edge) const;

    // TODO move somewhere else
    std::vector<size_t> offsets() const {
        std::vector<size_t> results;

        std::size_t index = 0;
        for (auto&& edge_info: edges) {
            results.push_back(index);
            assert(index + edge_info.node_count >= index);
            index += edge_info.node_count;
        }
        while(results.size() < edges.size() + 1)
            results.push_back(index);

        return results;
    }

    static std::vector<subdivision_edge_info> make_subdivision_info(
            const adjacency_list<int> &triangulation,
            const std::vector<node_t> &nodes,
            const polyhedron<adjacency_list<int>, 3> &polyhedron,
            const std::vector<double> &r_values,
            double epsilon);
};

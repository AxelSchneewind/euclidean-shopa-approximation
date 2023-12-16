#pragma once

#include "../graph/adjacency_list.h"
#include "polyhedron.h"

#include <vector>
#include <stdfloat>
#include <cstdint>
#include <cmath>
#include <cstdint>


class subdivision_table {
public:
    // in radians
    static constexpr int step_count = 127;

    // assume that each angle is at least 5 degrees
    static constexpr float min_angle = (M_PI / 180) *  5;
    static constexpr float max_angle = (M_PI / 180) * 85;

    static constexpr float step_size = (max_angle - min_angle) / (step_count - 1);

    struct edge_class {
        std::vector<float> node_positions;
    };

    struct subdivision_edge_info {
        // relative position of the points with the highest distance to other edges
        float mid_position;
        // maximum distance of a point on this edge to other edges, relative to the length of this edge
        // std::float16_t mid_dist;

        // number of steiner points on this edge (counting the source and middle node)
        unsigned short node_count;

        // r(v), relative to this edge
        float r_first;
        float r_second;

        // which classes of edges this one belongs to (first and second half)
        unsigned char edge_class_first;
        // interval of node positions
        unsigned short first_start_index;

        unsigned short mid_index;

        unsigned short second_start_index;
        unsigned char edge_class_second;

        bool operator==(const subdivision_edge_info &__other) const = default;
    };

    static_assert(sizeof(subdivision_edge_info) == 28);


    std::vector<edge_class> triangle_classes;
    std::vector<subdivision_edge_info> edges;

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(subdivision_edge_info);

    ~subdivision_table() = default;

    subdivision_table(subdivision_table &&__other) noexcept;

    subdivision_table(std::vector<edge_class> &&__node_positions, std::vector<subdivision_edge_info> &&__edges);

    static float class_angle(int __index);

    static int class_index(double __radians);

    coordinate_t node_coordinates(edge_id_t __edge, short steiner_index, coordinate_t c1, coordinate_t c2) const;

    subdivision_edge_info edge(int __edge) const;

    static std::vector<subdivision_table::edge_class> precompute(float __epsilon, float __min_relative_r_value);

    std::vector<size_t> offsets() const {
        std::vector<size_t> results;

        unsigned int index = 0;
        for (auto edge_info: edges) {
            results.push_back(index);
            index += edge_info.node_count;
        }
        results.push_back(index);
        results.push_back(index);

        return results;
    }

    static std::vector<float>
    min_r_per_triangle_class(const std::vector<node_t> &__nodes, const std::vector<float> &r_values,
                             const std::vector<triangle> &__faces);;


    static std::vector<subdivision_edge_info> make_subdivision_info(
            const adjacency_list<int, std::nullptr_t> &__triangulation,
            const std::vector<node_t> &__nodes,
            const polyhedron<adjacency_list<int, std::nullptr_t>, 3> &__polyhedron,
            const std::vector<edge_class> &__table,
            const std::vector<float> &__r_values,
            float __epsilon);
};

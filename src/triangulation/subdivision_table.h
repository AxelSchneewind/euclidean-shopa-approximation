#pragma once

#include "../graph/adjacency_list.h"
#include "polyhedron.h"
#include "steiner_graph.h"

#include <vector>
#include <stdfloat>
#include <cmath>


class subdivision_table {
public:
    // in radians
    static constexpr int step_count = 127;

    // assume that each angle is at least 10 degrees
    static constexpr float min_angle = (M_PI / 180) * 10;
    static constexpr float max_angle = (M_PI / 180) * 170;

    static constexpr float step_size = (max_angle - min_angle) / (step_count - 1);

    struct edge_class {
        std::vector<float> node_positions;
    };

    struct subdivision_edge_info {
        // relative position of the points with the highest distance to other edges
        std::float16_t mid_position;
        // maximum distance of a point on this edge to other edges, relative to the length of this edge
        // std::float16_t mid_dist;

        // r(v), relative to this edge
        std::float16_t r;

        // number of steiner points on this edge (counting the source and middle node)
        std::uint16_t node_count;

        // to which class of edges this one belongs
        unsigned char edge_class;

        // interval of node positions (last is first + node_count - 3)
        unsigned char first;

        bool operator==(const subdivision_edge_info &__other) const = default;
    };

    static_assert(sizeof(subdivision_edge_info) == 8);


    std::vector<edge_class> triangle_classes;
    std::vector<subdivision_edge_info> edges;

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(subdivision_edge_info);

    subdivision_table(subdivision_table &&__other) noexcept;;

    subdivision_table(std::vector<edge_class> &&__node_positions, std::vector<subdivision_edge_info> &&__edges);;

    static float class_angle(int __index);

    static int class_index(float __radians);

    coordinate_t node_coordinates(edge_id_t __edge, short steiner_index, coordinate_t c1, coordinate_t c2) const;

    subdivision_edge_info edge(int __edge) const;

    static std::vector<subdivision_table::edge_class> precompute(float __epsilon, float __min_relative_r_value);

    static std::vector<float>
    min_r_per_triangle_class(const std::vector<node_t> &__nodes, const std::vector<float> &r_values,
                             const std::vector<triangle> &__faces);;


    static std::vector<subdivision_edge_info> make_subdivision_info(
            const adjacency_list<int, std::nullptr_t> &__triangulation,
            const std::vector<node_t> &__nodes,
            const polyhedron<adjacency_list<int, std::nullptr_t>, 3> &__polyhedron,
            const std::vector<edge_class> &__table,
            const std::vector<std::float16_t> &__r_values,
            float __epsilon);
};

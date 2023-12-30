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
    static constexpr unsigned char step_count = 254;

    // assume that each angle is at least 5 degrees
    static constexpr double min_angle = (M_PI_2 / 90) *  5;
    static constexpr double max_angle = (M_PI_2 / 90) * 85;

    static constexpr double step_size = (max_angle - min_angle) / (step_count - 1);

    struct edge_class {
        std::vector<float> node_positions;
    };

    struct subdivision_edge_info {
        // relative position of the points with the highest distance to other edges
        float mid_position;
        // maximum distance of a point on this edge to other edges, relative to the length of this edge
        // std::float16_t mid_dist;

        // r(v), relative to this edge
        float r_first;
        float r_second;

        // number of steiner points on this edge (counting the source and middle node)
        unsigned short node_count;

        // which classes of edges this one belongs to (first and second half)
        unsigned char edge_class_first;
        unsigned char edge_class_second;

        // interval of node positions
        unsigned short first_start_index;
        unsigned short second_start_index;

        unsigned short mid_index;

        bool operator==(const subdivision_edge_info &__other) const = default;
    };


    std::vector<edge_class> triangle_classes;
    std::vector<subdivision_edge_info> edges;

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(subdivision_edge_info);

    ~subdivision_table() = default;

    subdivision_table(subdivision_table &&other) noexcept;

    subdivision_table(std::vector<edge_class> &&node_positions, std::vector<subdivision_edge_info> &&__edges);

    static double class_angle(int index);

    static int class_index(double radians);

    coordinate_t node_coordinates(edge_id_t edge, short steiner_index, coordinate_t const& c1, coordinate_t const& c2) const;

    subdivision_edge_info& edge(int edge);
    subdivision_edge_info const& edge(int edge) const;

    static std::vector<subdivision_table::edge_class> precompute(double epsilon, double __min_relative_r_value);

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
    min_r_per_triangle_class(const std::vector<node_t> &nodes, const std::vector<float> &r_values,
                             const std::vector<triangle> &__faces);


    static std::vector<subdivision_edge_info> make_subdivision_info(
            const adjacency_list<int, std::nullptr_t> &triangulation,
            const std::vector<node_t> &nodes,
            const polyhedron<adjacency_list<int, std::nullptr_t>, 3> &polyhedron,
            const std::vector<edge_class> &table,
            const std::vector<double> &r_values,
            double epsilon);
};

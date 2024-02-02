#include "subdivision_table.h"

#include "../graph/base_types.h"
#include "../util/is_in_range.h"
#include "subdivision_info.h"


#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>
#include <cmath>


std::vector<subdivision::subdivision_edge_info>
subdivision::make_subdivision_info(const adjacency_list<int> &triangulation,
                                        const std::vector<node_t> &nodes,
                                        const polyhedron<adjacency_list<int>, 3> &polyhedron,
                                        const std::vector<double> &r_values, double epsilon) {

    // store subdivision information here
    std::vector<subdivision_edge_info> result;
    result.reserve(triangulation.edge_count());

    for (size_t i = 0; i < triangulation.edge_count(); i++) {
        auto node1 = triangulation.source(i);
        auto node2 = triangulation.destination(i);

        // get coordinates
        auto const& c1 = nodes[node1].coordinates;
        auto const& c2 = nodes[node2].coordinates;

        // get minimal angle for node1 and node2
        // treat angles > 90 degrees like 90 degrees
        double angle1 = M_PI_2; // between node1->node2 and node1->node3
        double angle2 = M_PI_2; // between node2->node1 and node2->node3
        double angle3 = 0; // between node2->node1 and node2->node3

        for (auto&& edge: polyhedron.edges(i)) {
            if (is_none(edge)) continue;

            // make sure node3 is different from node1 and node2
            auto node3 = triangulation.destination(edge);
            if (node3 == node1 || node3 == node2)
                node3 = triangulation.source(edge);
            if (node3 == node1 || node3 == node2) continue;

            auto c3 = nodes[node3].coordinates;

            // compute angles
            auto a1 = inner_angle(c1, c2, c1, c3);
            auto a2 = inner_angle(c2, c1, c2, c3);
            auto a3 = inner_angle(c3, c1, c3, c2);
            assert(std::fabs((a1 + a2 + a3) - M_PI) <
                   M_PI / 180); // check that sum of inner angles is 180º (+- 1º for rounding)

            if (a1 < angle1)
                angle1 = a1;
            if (a2 < angle2)
                angle2 = a2;
        }
        angle1 = std::clamp(angle1, min_angle, M_PI_2);
        angle2 = std::clamp(angle2, min_angle, M_PI_2);
        angle3 = M_PI - angle2 - angle1;

        // length |e| of the edge
        double length = distance(c2, c1);

        // relative value where the mid-point (with max distance to other edges) lies between node1 and node2
        double mid_position = 1 / (1 + std::sin(angle1) / std::sin(angle2));
        {
            double mid_value_second = 1 / (1 + std::sin(angle2) / std::sin(angle1));
            assert(std::abs(mid_position + mid_value_second - 1.0) < 0.001);
            assert(mid_position < 1 && mid_position > 0);
        }

        // distance values have been computed already, convert to r(v) relative to this edges length
        double factor = std::min(epsilon, 1.0) / 5;
        double r_first = factor * (r_values[node1] / length);
        double r_second = factor * (r_values[node2] / length);
        r_first = std::clamp(r_first, min_r_value, 0.25);
        r_second = std::clamp(r_second, min_r_value, 0.25);

        // the base for computing relative node positions
        double base_first = std::clamp(1.0 + epsilon * std::sin(angle1), min_base, 2.0);
        double base_second = std::clamp(1.0 + epsilon * std::sin(angle2), min_base, 2.0);

        // get interval in first half that is between r and mid_value
        size_t left_count;
        {
            // exponential search
            left_count = 1;
            while (left_count < max_steiner_count_per_edge / 4 && std::pow(base_first, left_count) * r_first < mid_position)
                left_count *= 2;

            // binary search
            size_t max_count = left_count + 1;
            left_count /= 2;
            while (max_count > left_count) {
                if (std::pow(base_first, (max_count + left_count) / 2) * r_first < mid_position)
                    left_count = (max_count + left_count) / 2 + 1;
                else
                    max_count = (max_count + left_count) / 2;
            }

            // sanity check
            assert(left_count >= 0);
            auto relative = std::pow(base_first, (left_count-1)) * r_first;
            assert(left_count == 0 || left_count >= max_steiner_count_per_edge / 4 || relative >= r_first - 0.002F);
            assert(left_count == 0 || left_count >= max_steiner_count_per_edge / 4 || relative <= mid_position + 0.002F);
            relative = std::pow(base_first, (left_count)) * r_first;
            assert(left_count >= max_steiner_count_per_edge / 4 || relative >= mid_position - 0.002F);
        }

        // get interval in second edge half that is between r and mid_value
        size_t right_count;
        {
            // exponential search
            right_count = 1;
            while (right_count < max_steiner_count_per_edge / 4 && std::pow(base_second, right_count) * r_second < (1 - mid_position))
                right_count *= 2;

            // binary search
            size_t max_count = right_count + 1;
            right_count /= 2;
            while (max_count > right_count) {
                if (std::pow(base_second, (max_count + right_count) / 2) * r_second < (1 - mid_position))
                    right_count = (max_count + right_count) / 2 + 1;
                else
                    max_count = (max_count + right_count) / 2;
            }

            // sanity check
            assert(right_count >= 0);
            auto relative = std::pow(base_second, (right_count - 1)) * r_second;
            assert(right_count == 0 || right_count >= max_steiner_count_per_edge / 4 || relative >= r_second - 0.002F);
            assert(right_count == 0 || right_count >= max_steiner_count_per_edge / 4 || relative <= (1 - mid_position) + 0.002F);
            relative = std::pow(base_second, (right_count)) * r_second;
            assert(right_count >= max_steiner_count_per_edge / 4 || relative >= (1 - mid_position) - 0.002F);
        }

        // number of points on first half of edge
        auto mid_index = left_count + 1; // c1, steiner points
        // remove point at r(v) if already over on other half of the edge
        if (r_first >= mid_position && left_count > 0) {
            r_first = mid_position;
            mid_index--;
        }
        {
            auto relative = std::pow(base_first, mid_index - 2) * r_first;
            assert(mid_index < 2 || relative >= r_first - 0.002F);
            assert(mid_index < 2 || relative <= mid_position + 0.002F);
        }

        // number of points (points on first half + mid_node + points on second half + c2
        auto count = 1 + left_count + 1 + right_count + 1;
        // remove point at r(v) if already over on other half of the edge
        if (r_second >= 1 - mid_position) {
            r_second = 1 - mid_position;
            count--;
        }
        assert(count >= 2);

        // check that values are in bounds
        if (!is_in_range(count, 2, max_steiner_count_per_edge)
            || !is_in_range(mid_position, 0.0, 1.0)
            || !is_in_range(mid_index, 1, count)
            || !is_in_range(r_first,  0, 0.5)
            || !is_in_range(r_second, 0, 0.5)
            || !is_in_range(left_count,  0, max_steiner_count_per_edge / 2)
            || !is_in_range(right_count, 0, max_steiner_count_per_edge / 2))
            throw std::invalid_argument("some value does not fit");

        result.emplace_back();
        auto& entry = result.back();
        entry.mid_position = mid_position;
        entry.mid_index = mid_index;
        entry.node_count = count;
        entry.r_first = r_first;
        entry.r_second = r_second;
        entry.base_first = base_first;
        entry.base_second = base_second;
    }

    return result;
}

coordinate_t
subdivision::node_coordinates(edge_id_t edge, steiner_index_type steiner_index, coordinate_t const &c1,
                                    coordinate_t const &c2) const {
    auto &&info = edges[edge];

    assert(steiner_index >= 0);
    assert(steiner_index < info.node_count);

    if (steiner_index == 0) [[unlikely]]
        return c1;
    else if (steiner_index == info.node_count - 1) [[unlikely]]
        return c2;

    if (steiner_index == info.mid_index) [[unlikely]]
        return interpolate_linear(c1, c2, info.mid_position);

    // if (steiner_index == 1) [[unlikely]]
    //     return interpolate_linear(c1, c2, info.r_first);
    // if (steiner_index == info.node_count - 2) [[unlikely]]
    //     return interpolate_linear(c2, c1, info.r_second);

    if (steiner_index < info.mid_index) [[likely]] {
        auto index = steiner_index - 1;
        assert(index >= 0);

        auto &&relative = std::pow(info.base_first, index) * info.r_first;
        assert(index != 0 || relative == info.r_first);
        assert(relative >= info.r_first - 0.002F);
        assert(relative <= info.mid_position + 0.002F);
        return interpolate_linear(c1, c2, relative);
    }
    // if (steiner_index > info.mid_index) [[likely]] {
        auto index = info.node_count - steiner_index - 2;
        assert(index >= 0);

        auto &&relative = std::pow(info.base_second, index) * info.r_second;
        assert(steiner_index != info.node_count - 2 || index == 0);
        assert(index != 0 || relative == info.r_second);
        assert(relative >= info.r_second - 0.002F);
        assert(relative <= 1.0F - info.mid_position + 0.002F);
        return interpolate_linear(c2, c1, relative);
    // }
}

subdivision::subdivision_edge_info &subdivision::edge(edge_id_t edge) {
    return edges[edge];
}

const subdivision::subdivision_edge_info &subdivision::edge(edge_id_t edge) const {
    return edges[edge];
}
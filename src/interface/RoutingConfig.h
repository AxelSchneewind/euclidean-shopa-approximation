#pragma once

struct RoutingConfiguration {
    bool use_a_star{true};
    bool bidirectional{false};
    bool live_status {true};
    enum {
        LINALG,
        BINSEARCH,
        ATAN2
    } min_angle_neighbor_method;
    std::size_t tree_size{0};
};

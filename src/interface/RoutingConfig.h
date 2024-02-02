#pragma once

struct RoutingConfiguration {
    bool use_a_star{true};
    bool bidirectional{false};
    bool compact_labels{true};
    bool live_status {true};
    std::size_t tree_size{0};
};

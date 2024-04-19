#pragma once

struct RoutingConfiguration {
    bool use_a_star{true};
    bool live_status {true};
    bool only_distance{false};
    bool store_coords {false};
    enum class Pruning {
        UNPRUNED,
        PRUNE_DEFAULT,
        MinBendingAngleESpanner
    } pruning;
    enum class NeighborFindingAlgorithm {
        PARAM,
        ATAN2,
        BINSEARCH,
        LINEAR
    } neighbor_selection_algorithm;
    std::size_t tree_size{0};
};

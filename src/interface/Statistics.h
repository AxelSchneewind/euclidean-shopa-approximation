#pragma once


enum Statistics {
    NODE_COUNT,
    EDGE_COUNT,
    STORED_NODE_COUNT,
    STORED_EDGE_COUNT,
    EPSILON,
    MEMORY_USAGE_GRAPH,
    MEMORY_USAGE_FINAL,
    FROM,
    TO,
    FROM_INTERNAL,
    TO_INTERNAL,
    COST,
    BEELINE_DISTANCE,
    EPSILON_SATISFIED,
    TREE_SIZE,
    QUEUE_PULL_COUNT,
    QUEUE_PUSH_COUNT,
    QUEUE_MAX_SIZE,
    EDGES_CHECKED,
    TIME,
    NEIGHBORS_BASE_NODE_COUNT,
    NEIGHBORS_BASE_NODE_NEIGHBORS_COUNT,
    NEIGHBORS_BOUNDARY_NODE_COUNT,
    NEIGHBORS_BOUNDARY_NODE_NEIGHBORS_COUNT,
    NEIGHBORS_STEINER_POINT_COUNT,
    NEIGHBORS_STEINER_POINT_NEIGHBORS_COUNT,
    PATH,
    NUM_COLUMNS
};

const std::array<std::string, NUM_COLUMNS> COLUMNS{
    "NODE COUNT",
    "EDGE COUNT",
    "STORED NODE COUNT",
    "STORED EDGE COUNT",
    "EPSILON",
    "MEMORY USAGE GRAPH",
    "MEMORY USAGE FINAL",
    "FROM",
    "TO",
    "FROM INTERNAL",
    "TO INTERNAL",
    "COST",
    "BEELINE DISTANCE",
    "EPSILON SATISFIED",
    "TREE SIZE",
    "QUEUE PULL COUNT",
    "QUEUE PUSH COUNT",
    "QUEUE MAX SIZE",
    "EDGES CHECKED",
    "TIME",
    "NEIGHBORS BASE NODE COUNT",
    "NEIGHBORS BASE NODE NEIGHBORS COUNT",
    "NEIGHBORS BOUNDARY NODE COUNT",
    "NEIGHBORS BOUNDARY NODE NEIGHBORS COUNT",
    "NEIGHBORS STEINER POINT COUNT",
    "NEIGHBORS STEINER POINT NEIGHBORS COUNT",
    "PATH"
};

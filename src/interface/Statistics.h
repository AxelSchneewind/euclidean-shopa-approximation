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
    TIME,
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
        "TIME",
        "PATH"
};

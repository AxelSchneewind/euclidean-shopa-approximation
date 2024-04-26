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
    FROM_LAT,
    FROM_LON,
    TO_LAT,
    TO_LON,
    COST,
    TIME,
    ASTAR,
    NEIGHBOR_FINDING,
    PRUNING,
    BEELINE_DISTANCE,
    TREE_SIZE,
    QUEUE_PULL_COUNT,
    QUEUE_PUSH_COUNT,
    QUEUE_MAX_SIZE,
    EDGES_CHECKED,
    NEIGHBORS_BASE_NODE_COUNT,
    NEIGHBORS_BASE_NODE_NEIGHBORS_COUNT,
    NEIGHBORS_BOUNDARY_NODE_COUNT,
    NEIGHBORS_BOUNDARY_NODE_NEIGHBORS_COUNT,
    NEIGHBORS_STEINER_POINT_COUNT,
    NEIGHBORS_STEINER_POINT_NEIGHBORS_COUNT,
    NEIGHBORS_STEINER_POINT_ANGLE_CHECK_COUNT,
    GRAPH_FILE,
    PATH,
    NUM_COLUMNS
};

const std::array<std::string, NUM_COLUMNS> COLUMNS{
    "node count",
    "edge count",
    "stored node count",
    "stored edge count",
    "epsilon",
    "memory usage graph",
    "memory usage final",
    "source",
    "target",
    "source internal",
    "target internal",
    "source latitude",
    "source longitude",
    "target latitude",
    "target longitude",
    "cost",
    "time",
    "astar",
    "neighbor finding algorithm",
    "pruning",
    "beeline distance",
    "tree size",
    "queue pull count",
    "queue push count",
    "queue max size",
    "edges checked",
    "neighbors base node count",
    "neighbors base node neighbors count",
    "neighbors boundary node count",
    "neighbors boundary node neighbors count",
    "neighbors steiner point count",
    "neighbors steiner point neighbors count",
    "neighbors steiner point search iteration count",
    "graph",
    "path"
};

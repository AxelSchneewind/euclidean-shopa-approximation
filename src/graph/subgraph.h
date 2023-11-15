#include <vector>

template<typename NodeId>
struct path {
    std::vector<NodeId> nodes;
};

template<typename NodeId, typename EdgeId>
struct subgraph {
    std::vector<NodeId> nodes;
    std::vector<EdgeId> edges;

    subgraph() = default;

    subgraph(std::vector<NodeId> &&__n, std::vector<EdgeId> &&__e);
};

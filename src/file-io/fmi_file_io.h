#pragma once

#include "formatters.h"
#include "../graph/graph.h"

#include <istream>
#include <ostream>
#include <span>
#include <vector>

class fmi_file_io {
public:

    template<typename NodeInfo, typename formatter>
    static std::vector<NodeInfo>
    read_nodes(std::istream &input, std::size_t count);

    template<typename NodeInfo, typename formatter>
    static std::vector<NodeInfo>
    read_nodes(std::istream &input, std::size_t count, float coordinate_factor);

    template<typename NodeInfo, typename formatter, typename Projection>
    static std::vector<NodeInfo>
    read_nodes(std::istream &input, std::size_t count, Projection &projection);

    template<typename NodeInfo, typename NodeId, typename EdgeInfo, typename formatter>
    static auto
    read_edges(std::istream &input, std::vector<NodeInfo> const &nodes, std::size_t count);

    template<typename NodeInfo, typename NodeId, typename EdgeInfo, typename formatter>
    static auto
    read_edges(std::istream &input, std::vector<NodeInfo> const &nodes, std::size_t count, float cost_factor);

    template<typename NodeInfo, typename NodeId, typename EdgeInfo, typename formatter, typename DistanceProjection>
    static auto
    read_edges(std::istream &input, std::vector<NodeInfo> const &nodes, std::size_t count,
               DistanceProjection &projection);

    template<typename Graph, typename format = stream_encoders::encode_text>
    static Graph read(std::istream &input) { return read<Graph, format>(input, input, input); };

    template<typename Graph, typename format = stream_encoders::encode_text>
    static Graph read(std::istream &input_sizes, std::istream &input_nodes, std::istream &input_edges);

    template<typename Graph, typename format = stream_encoders::encode_text>
    static std::ostream &write(std::ostream &output, const Graph &graph);
};

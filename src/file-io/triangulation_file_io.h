#pragma once

#include "../graph/graph.h"
#include "../triangulation/steiner_graph.h"
#include "formatters.h"


class triangulation_file_io {
public:


    template<typename NodeInfo, typename NodeId, typename EdgeInfo, typename formatter>
    static auto
    read_triangles(std::istream &input, std::vector<NodeInfo> const& nodes, std::size_t count, std::vector<std::array<NodeId, 3>>& faces);

    template<typename NodeInfo, typename NodeId, typename EdgeInfo, typename formatter>
    static auto
    read_triangles(std::istream &input, std::vector<NodeInfo> const& nodes, std::size_t count);


    template<Topology Graph, typename format = stream_encoders::encode_text>
    static Graph read(std::istream &input) { return read<Graph, format> (input, input, input); };
    template<Topology Graph, typename format = stream_encoders::encode_text>
    static Graph read(std::istream &input_size, std::istream& input_nodes, std::istream& input_edges);

    static steiner_graph read_steiner(std::istream &input_size, std::istream& input_nodes, std::istream& input_triangles, float __epsilon);
    static steiner_graph read_steiner(std::istream &input, float __epsilon) { return read_steiner(input, input, input, __epsilon); };

    template<Topology Graph, typename format = stream_encoders::encode_text>
    static void write(std::ostream &output, const Graph &graph);
};


#pragma once

#include "formatters.h"

#include <iostream>
#include <vector>

namespace file_io {

    template<typename NodeInfo, typename formatter=stream_encoders::encode_text>
    void
    read_nodes(std::istream &input, std::span<NodeInfo> nodes);

    template<typename NodeInfo, typename formatter=stream_encoders::encode_text>
    void
    write_nodes(std::ostream &output, std::span<NodeInfo> nodes);

    template<typename Edge, typename formatter=stream_encoders::encode_text>
    void
    read_edges(std::istream &input, std::span<Edge> edges);

    template<typename Edge, typename formatter=stream_encoders::encode_text>
    void
    write_edges(std::ostream &output, std::span<Edge> edges);

    template<typename NodeId, typename formatter=stream_encoders::encode_text>
    void
    read_triangles(std::istream &input, std::span<std::array<NodeId, 3>> faces);

    template<typename NodeId, typename formatter=stream_encoders::encode_text>
    void
    write_triangles(std::ostream &output, std::span<std::array<NodeId, 3>> faces);

}
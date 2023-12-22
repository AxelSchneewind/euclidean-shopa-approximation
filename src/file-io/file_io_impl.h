#pragma once

#include "file_io.h"

template<typename NodeInfo, typename formatter>
void
file_io::read_nodes(std::istream &input, std::span<NodeInfo> nodes) {
    for (auto& n : nodes) {
        n = formatter::template read<NodeInfo>(input) ;
    }
}


template<typename NodeInfo, typename formatter>
void file_io::write_nodes(std::ostream &output, std::span<NodeInfo> nodes) {
    for (int i = 0; i < nodes.size(); ++i) {
        NodeInfo const &n = nodes[i];
        formatter::write(output, n);
        formatter::write(output, '\n');
    }

}

template<typename Edge, typename formatter>
void
file_io::read_edges(std::istream &input, std::span<Edge> edges) {
    for (auto& edge : edges) {
        edge = formatter::template read<Edge>(input);
    }
}

template<typename Edge, typename formatter>
void
file_io::write_edges(std::ostream &output, std::span<Edge> edges) {
    for (auto const& edge : edges) {
        formatter::template write(output, edge.source);
        formatter::write(output, ' ');
        formatter::template write(output, edge.destination);
        formatter::write(output, ' ');
        formatter::template write(output, edge.info);
        formatter::write(output, '\n');
    }
}

template<typename NodeId, typename formatter>
void
file_io::read_triangles(std::istream &input, std::span<std::array<NodeId, 3>> faces) {
    for (auto& face : faces)
        face = formatter::template read<std::array<NodeId, 3>>(input);
}


template<typename NodeId, typename formatter>
void file_io::write_triangles(std::ostream &output, std::span<std::array<NodeId, 3>> faces) {
    for(auto&& face : faces) {
        for (auto& id : face) {
            formatter::template write<NodeId>(output, id);
            formatter::write(output, ' ');
        }
        formatter::write(output, '\n');
    }
}

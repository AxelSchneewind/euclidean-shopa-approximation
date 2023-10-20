#pragma once

#include "../graph/base_types.h"
#include "../graph/graph.h"
#include "../routing/routing.h"
#include <istream>
#include <ostream>
#include <span>
#include <vector>

typedef graph_t<node_t, edge_t> standard_graph_t;
typedef routing_t<node_t, edge_t> standard_routing_t;

typedef graph_t<ch_node_t, ch_edge_t> ch_graph_t;
typedef routing_t<ch_node_t, ch_edge_t> ch_routing_t;


enum file_format { fmi,
                   fmi_ch,
                   binary,
                   binary_ch,
                   binary_compressed,
                   binary_ch_compressed };

std::istream &skip_comments(std::istream &s);

/*// functions to define for each file format and datatype
template <file_format format, typename T>
T read(std::istream &input);

template <file_format format, typename T>
std::ostream &write(std::ostream &output, const T &value);

template <file_format format, typename T>
std::vector<T> read(std::istream &input, int count)

template <file_format format, typename T>
std::ostream &write(std::ostream &output, std::span<const T> &values);
*/

class fmi_format {
public:
    template<typename node_info, typename edge_info>
    graph_t<node_info, edge_info> read(std::istream &input);

    template<typename node_info, typename edge_info>
    std::ostream &write(std::ostream &output, const graph_t<node_info, edge_info>& graph);
};


class binary_format {
public:
    template<typename node_info, typename edge_info>
    graph_t<node_info, edge_info> read(std::istream &input);

    template<typename node_info, typename edge_info>
    std::ostream &write(std::ostream &output, const graph_t<node_info, edge_info>& graph);
};


namespace {
namespace format_text {

    template<typename T>
    T read(std::istream &input);
    template<typename T>
    std::ostream &write(std::ostream &output, const T &value);

    // read/write lists
    template<typename T>
    std::vector<T> read(std::istream &input, int count);

    template<typename T>
    std::ostream &write(std::ostream &output, std::span<const T> &values);

};// namespace format_text

namespace format_binary {

    template<typename T>
    T read(std::istream &input);

    template<typename T>
    std::ostream &write(std::ostream &output, const T &value);

    template<typename T>
    std::vector<T> read(std::istream &input, int count);

    template<typename T>
    std::ostream &write(std::ostream &output, std::span<const T> &values);

};// namespace format_binary






}


long ignore;

std::istream &
skip_comments(std::istream &s) {
    s >> std::ws;
    while (s.peek() == '#' || s.peek() == '\n') {
        s.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        s >> std::ws;
    }
    return s;
}


/*// functions to define for each file format and datatype
template <file_format format, typename T>
T read(std::istream &input);

template <file_format format, typename T>
std::ostream &write(std::ostream &output, const T &value);

template <file_format format, typename T>
std::vector<T> read(std::istream &input, int count)

template <file_format format, typename T>
std::ostream &write(std::ostream &output, std::span<const T> &values);
*/


template<typename T>
T
format_text::read(std::istream &input) {
    T result;
    input >> result;
    return result;
}

template<typename T>
std::ostream &
format_text::write(std::ostream &output, const T &value) {
    output << value;
    return output;
}

// standard nodes and edges
template<>
node_t
format_text::read(std::istream &input) {
    node_t result;
    input >> ignore >> ignore >> result.coordinates.latitude >> result.coordinates.longitude >> ignore;
    return result;
}

template<>
std::ostream &
format_text::write(std::ostream &output, const node_t &node) {
    output << 0 << 0 << node.coordinates.latitude << node.coordinates.longitude << 0;
    return output;
}

template<>
adjacency_list_edge_t<edge_t>
format_text::read(std::istream &input) {
    adjacency_list_edge_t<edge_t> result;
    input >> result.source >> result.destination >> result.info.cost >> ignore >> ignore;
    return result;
}

template<>
std::ostream &
format_text::write(std::ostream &output, const adjacency_list_edge_t<edge_t> &edge) {
    output << 0 << edge.destination << edge.info.cost << 0 << 0;
    return output;
}



// CH nodes and edges
template<>
ch_node_t
format_text::read(std::istream &input) {
    ch_node_t result;
    input >> ignore >> ignore >> result.coordinates.latitude >> result.coordinates.longitude >> ignore >> result.level;
    return result;
}

template<>
std::ostream &
format_text::write(std::ostream &output, const ch_node_t &node) {
    output << node.coordinates.latitude << node.coordinates.longitude << 0 << node.level;
    return output;
}

template<>
adjacency_list_edge_t<ch_edge_t>
format_text::read(std::istream &input) {
    adjacency_list_edge_t<ch_edge_t> result;
    input >> result.source >> result.destination >> result.info.cost >> ignore >> ignore >> result.info.edgeA >> result.info.edgeB;
    return result;
}

template<>
std::ostream &
format_text::write(std::ostream &output, const adjacency_list_edge_t<ch_edge_t> &edge) {
    output << edge.source << edge.destination << edge.info.cost << 0 << 0 << edge.info.edgeA << edge.info.edgeB;
    return output;
}

// read/write lists
template<typename T>
std::vector<T>
format_text::read(std::istream &input, int count) {
    std::vector<T> result;
    result.reserve(count);

    for (size_t i = 0; i < count; i++)
        result.emplace_back(read<T>(input));

    return result;
}

template<typename T>
std::ostream &
format_text::write(std::ostream &output, std::span<const T> &values) {
    for (size_t i = 0; i < values.extent(); i++)
        write<T>(output, values[i]);
    return output;
}

template<typename T>
T
format_binary::read(std::istream &input) {
    T result;
    input.read((char *) &result, sizeof(T));
    return result;
}

template<typename T>
std::ostream &
format_binary::write(std::ostream &output, const T &value) {
    output.write((char *) &value, sizeof(T));
    return output;
}

template<typename T>
std::vector<T>
format_binary::read(std::istream &input, int count) {
    std::vector<T> result(count);
    input.read((char *) result.data(), sizeof(T) * count);
    return result;
}

template<typename T>
std::ostream &
format_binary::write(std::ostream &output, std::span<const T> &values) {
    output.write((char *) values.begin(), sizeof(T) * values.extent());
    return output;
}


template<typename node_info, typename edge_info>
graph_t<node_info, edge_info>
fmi_format::read(std::istream &input) {
    node_id_t node_count(format_text::read<node_id_t>(input));
    edge_id_t edge_count(format_text::read<edge_id_t>(input));

    std::vector<node_info> nodes(format_text::read<node_info>(input, node_count));

    typename adjacency_list<edge_info>::adjacency_list_builder builder;
    builder.add_node(node_count - 1);

    for (edge_id_t edge_index = 0; edge_index < edge_count; edge_index++)
        builder.add_edge(format_text::read<adjacency_list_edge_t<edge_info> >(input));

    std::shared_ptr<adjacency_list<edge_info>> forward(new adjacency_list<edge_info>(builder.get()));
    return graph_t<node_info, edge_info>(node_count, edge_count, std::move(nodes), forward);
}

template<typename node_info, typename edge_info>
std::ostream &
fmi_format::write(std::ostream &output, const graph_t<node_info, edge_info>& graph) {
    node_id_t node_count = graph.node_count();
    node_id_t edge_count = graph.edge_count();

    format_text::write(output, node_count);
    format_text::write(output, edge_count);

    for (int n = 0; n < node_count; ++n) {
        format_text::write(output, graph.node(n));
    }

    for (edge_id_t edge_index = 0; edge_index < edge_count; edge_index++) {
        format_text::write(output, graph.forward().adjlist_edge(edge_index));
    }

    return output;
}


template<typename node_info, typename edge_info>
graph_t<node_info, edge_info>
binary_format::read(std::istream &input) {
    node_id_t node_count = format_binary::read<node_id_t>(input);
    edge_id_t edge_count = format_binary::read<edge_id_t>(input);

    std::vector<node_info> nodes = format_binary::read<node_info>(input, node_count);

    std::shared_ptr<adjacency_list<edge_info>> forward_edges(new adjacency_list<edge_info>(node_count, edge_count));
    for (edge_id_t edge_index = 0; edge_index < edge_count; edge_index++)
        forward_edges->push_back_edge(format_binary::read<adjacency_list_edge_t<edge_info> >(input));

    forward_edges->push_back_node(node_count - 1);

    return graph_t<node_info, edge_info>(node_count, edge_count, std::move(nodes), forward_edges);
}

template<typename node_info, typename edge_info>
std::ostream &
binary_format::write(std::ostream &output, const graph_t<node_info, edge_info>& graph) {
    node_id_t node_count = graph.node_count();
    node_id_t edge_count = graph.forward_list.edge_count();
    format_binary::write(output, node_count);
    format_binary::write(output, edge_count);

    for (int n = 0; n < node_count; ++n) {
        format_binary::write(output, graph.node(n));
    }

    for (edge_id_t edge_index = 0; edge_index < edge_count; edge_index++) {
        format_binary::write(output, graph.forward().edge(edge_index));
    }

    return output;
}

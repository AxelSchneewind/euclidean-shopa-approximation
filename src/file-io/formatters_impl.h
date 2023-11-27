#pragma once

#include "formatters.h"

#include "../graph/base_types.h"
#include "../graph/unidirectional_adjacency_list.h"
#include "../triangulation/steiner_graph.h"

#include <iostream>
#include <vector>
#include <span>

namespace stream_encoders {

	static long ignore;

    std::istream &
    encode_text::skip_comments(std::istream &s) {
        s >> std::ws;
        while (s.peek() == '#' || s.peek() == '\n') {
            s.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            s >> std::ws;
        }
        return s;
    }

    template<typename T>
    T
    encode_text::read(std::istream &input) {
        T result;
        input >> result;
        return result;
    }

    template<typename T>
    std::ostream &
    encode_text::write(std::ostream &output, const T &value) {
        output << value;
        return output;
    }


// coordinates
    template<>
    coordinate_t
    encode_text::read(std::istream &input) {
        coordinate_t result;
        input >> result.latitude >> result.longitude;
        return result;
    }

    template<>
    std::ostream &
    encode_text::write(std::ostream &output, const coordinate_t &c) {
        output << c.latitude << ' ' << c.longitude << ' ';
        return output;
    }

// standard nodes and edges
    template<>
    node_t
    encode_text::read(std::istream &input) {
        node_t result;
        input >> result.coordinates.latitude >> result.coordinates.longitude;
        return result;
    }

    template<>
    std::ostream &
    encode_text::write(std::ostream &output, const node_t &node) {
        output << node.coordinates.latitude << ' ' << node.coordinates.longitude;
        return output;
    }



    template<>
    adjacency_list_edge<node_id_t, edge_t>
    encode_text::read(std::istream &input) {
        adjacency_list_edge<node_id_t, edge_t> result;
        input >> result.source >> result.destination >> result.info.cost;
        return result;
    }

    template<>
    std::ostream &
    encode_text::write(std::ostream &output, const adjacency_list_edge<node_id_t, edge_t> &edge) {
        output << (int) edge.source << ' ' << (int) edge.destination << ' ' << (float) edge.info.cost;
        return output;
    }

    template<>
    std::ostream &
    encode_text::write(std::ostream &output, edge_t const&edge) {
        output << (float) edge.cost;
        return output;
    }

    template<>
    std::ostream &
    encode_text::write(std::ostream &output, ch_edge_t const&edge) {
        output << (float) edge.cost << ' ' << edge.edgeA << ' ' << edge.edgeB;
        return output;
    }


    template<>
    adjacency_list_edge<node_id_t, ch_edge_t>
    encode_text::read(std::istream &input) {
        adjacency_list_edge<node_id_t, ch_edge_t> result;
        input >> result.source >> result.destination >> result.info.cost;
        return result;
    }

    template<>
    std::ostream &
    encode_text::write(std::ostream &output, const adjacency_list_edge<node_id_t, ch_edge_t> &edge) {
        output << (int) edge.source << ' ' << (int) edge.destination << ' ' << (float) edge.info.cost << ' ' << edge.info.edgeA << ' ' << edge.info.edgeB;
        return output;
    }

// triangles
    template<>
    triangle
    encode_text::read(std::istream &input) {
        triangle result;
        input >> result[0] >> result[1] >> result[2];
        return result;
    }

    template<>
    std::ostream &
    encode_text::write(std::ostream &output, const triangle &triangle) {
        output << triangle[0] << ' ' << triangle[1] << ' ' << triangle[2] << ' ';
        return output;
    }

// steiner node and edge ids
    template<>
    std::ostream &
    encode_text::write(std::ostream &output, const steiner_node_id &value) {
        output << (int) value.edge << ':' << (int) value.steiner_index << ' ';
        return output;
    }

    template<>
    std::ostream &
    encode_text::write(std::ostream &output, const steiner_edge_id &value) {
        output << value.source << ':' << value.source << ' ';
        return output;
    }


    // CH nodes and edges
    template<>
    ch_node_t
    encode_text::read(std::istream &input) {
        ch_node_t result;
        input >> ignore >> ignore >> result.coordinates.latitude >> result.coordinates.longitude >> ignore
              >> result.level;
        return result;
    }

    template<>
    std::ostream &
    encode_text::write(std::ostream &output, const ch_node_t &node) {
        output << node.coordinates.latitude << ' ' << node.coordinates.longitude << ' ' << node.level
               << ' ';
        return output;
    }



// read/write lists
    template<typename T>
    std::vector<T>
    encode_text::read(std::istream &input, int count) {
        std::vector<T> result;
        result.reserve(count);

        for (size_t i = 0; i < count; i++)
            result.emplace_back(read<T>(input));

        return result;
    }

    template<typename T>
    std::ostream &
    encode_text::write(std::ostream &output, std::span<const T> &values) {
        for (size_t i = 0; i < values.extent(); i++)
            write<T>(output, values[i]);
        return output;
    }

    template<typename T>
    T
    encode_binary::read(std::istream &input) {
        T result;
        input.read((char *) &result, sizeof(T));
        return result;
    }

    template<typename T>
    std::ostream &
    encode_binary::write(std::ostream &output, const T &value) {
        output.write((char *) &value, sizeof(T));
        return output;
    }

    template<typename T>
    std::vector<T>
    encode_binary::read(std::istream &input, int count) {
        std::vector<T> result(count);
        input.read((char *) result.data(), sizeof(T) * count);
        return result;
    }

    template<typename T>
    std::ostream &
    encode_binary::write(std::ostream &output, std::span<const T> &values) {
        output.write((char *) values.begin(), sizeof(T) * values.extent());
        return output;
    }

} // namespace stream_encoders

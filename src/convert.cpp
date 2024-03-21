#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

// number of digits of coordinates to write to output files
static constexpr int coordinate_precision = 20;

/*
 * specification for the .node format:
 *    First line:      <# of vertices> <dimension (must be 2)> <# of attributes> <# of boundary markers (0 or 1)>
 *    Remaining lines: <vertex #> <x> <y> [attributes] [boundary marker]
 *
 * specification for the .ele format:
 *    First line: <# of triangles> <nodes per triangle> <# of attributes>
 *    Remaining lines: <triangle #> <node> <node> <node> ... [attributes]
 *
 * specification of the .graph format:
 *    First line: <# of vertices>
 *    Second line: <# of triangles>
 *    Remaining lines (per vertex): <x> <y>
 *    Remaining lines (per triangle): <node> <node> <node>
 */


// write header of .node files
void
write_node_header(size_t num_vertices, int dimension, int num_attributes, bool boundary_marker, std::ostream &output) {
    output << num_vertices << ' ' << dimension << ' ' << num_attributes << ' ' << boundary_marker << '\n';
}

// write header of .ele files
void write_ele_header(size_t num_triangles, int nodes_per_triangle, int num_attributes, std::ostream &output) {
    output << num_triangles << ' ' << nodes_per_triangle << ' ' << num_attributes << '\n';
}

// write header of .graph files
void write_graph_header(size_t num_vertices, size_t num_triangles, std::ostream &output) {
    output << num_vertices << '\n' << num_triangles << '\n';
}

// read header of .node files
void read_node_header(size_t &num_vertices, int &dimension, int &num_attributes, bool &boundary_marker,
                      std::istream &input) {
    input >> num_vertices >> dimension >> num_attributes >> boundary_marker;
}

// read header of .ele files
void read_ele_header(size_t &num_triangles, int &nodes_per_triangle, int &num_attributes, std::istream &input) {
    input >> num_triangles >> nodes_per_triangle >> num_attributes;
}

// read header of .graph files
void read_graph_header(size_t &num_vertices, size_t &num_triangles, std::istream &input) {
    input >> num_vertices >> num_triangles;
}

/**
 * Reads vertices from the given input stream (containing only x and y coordinates) and converts to the .node format.
 * @param boundary_marker controls whether a boundary marker column (containing 1s) is added.
 */
void convert_graph_to_node(std::ifstream &input, std::ofstream &output, size_t count, bool boundary_marker) {
    output << std::setprecision(coordinate_precision);
    for (size_t i = 0; i < count; ++i) {
        double x, y;
        input >> x >> y;
        if (boundary_marker) {
            output << i << ' ' << x << ' ' << y << " 1" << '\n';
        } else {
            output << i << ' ' << x << ' ' << y << '\n';
        }
    }
}

/**
 * Reads vertices from the given input stream (.node format) and outputs only x and y coordinates.
 * Assumes vertices in .node file are ordered by id.
 * @param boundary_marker whether the file contains a boundary marker column.
 */
void convert_node_to_graph(std::ifstream &input, std::ofstream &output, size_t count, bool boundary_marker) {
    output << std::setprecision(coordinate_precision);
    for (size_t i = 0; i < count; ++i) {
        size_t vertex_num;
        bool boundary;
        double x, y;
        if (boundary_marker) {
            input >> vertex_num >> x >> y >> boundary;
        } else {
            input >> vertex_num >> x >> y;
        }

        if (vertex_num != i) {
            std::cerr << "vertices are not ordered linearly (vertex " << vertex_num << " at index " << i << ")\n";
        }

        output << x << ' ' << y << '\n';
    }
}

/*
 * Reads from the given input stream (containing 3 vertex ids per line) and converts to the .ele format.
 */
void convert_graph_to_ele(std::ifstream &input, std::ofstream &output, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        size_t id1, id2, id3;
        input >> id1 >> id2 >> id3;
        output << i << ' ' << id1 << ' ' << id2 << ' ' << id3 << '\n';
    }
}

/*
 * Reads from the given input stream (.ele format) and writes to a .graph file (three indices per line).
 */
void convert_ele_to_graph(std::ifstream &input, std::ofstream &output, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        size_t index, id1, id2, id3;
        input >> index >> id1 >> id2 >> id3;
        output << id1 << ' ' << id2 << ' ' << id3 << '\n';
    }
}


void print_help() {
    std::cout << "usage: convert path/to/input.graph path/to/output.node path/to/output.ele\n"
                 "or     convert path/to/input.node  path/to/input.ele   path/to/output.graph\n";
}


// function that replaces std::string::ends_with which is introduced in C++20
bool suffix_equals(std::string const& str, std::string const& suffix) {
    std::string actual_suffix = str.substr(str.size() - suffix.size(), suffix.size() );
    return actual_suffix == suffix;
}


int main(int argc, char *argv[]) {
    if (argc < 4) {
        print_help();
        return 1;
    }

    std::string file1{argv[1]};
    std::string file2{argv[2]};
    std::string file3{argv[3]};

    if (suffix_equals(file1, ".graph") && suffix_equals(file2, ".node") && suffix_equals(file3, ".ele")) {
        std::ifstream input(file1);
        std::ofstream output_node(file2);
        std::ofstream output_ele(file3);

        if (input.bad() || output_node.bad() || output_ele.bad()) {
            std::cout << "some file cannot be written to or read from\n";
            return 1;
        }

        size_t vertex_count, triangle_count;
        int dimension { 2 }, num_vertex_attributes { 0 }, num_triangle_attributes { 0 }, nodes_per_triangle { 3 };
        bool boundary_marker = 0;

        read_graph_header(vertex_count, triangle_count, input);

        write_node_header(vertex_count, dimension, num_vertex_attributes, boundary_marker, output_node);
        convert_graph_to_node(input, output_node, vertex_count, boundary_marker);

        write_ele_header(triangle_count, nodes_per_triangle, num_triangle_attributes, output_ele);
        convert_graph_to_ele(input, output_ele, triangle_count);
    } else if (suffix_equals(file1, ".node") && suffix_equals(file2, ".ele") && suffix_equals(file3, ".graph")) {
        std::ifstream input_node(file1);
        std::ifstream input_ele(file2);
        std::ofstream output(file3);

        if (input_node.bad() || input_ele.bad() || output.bad()) {
            std::cout << "some file cannot be written to or read from\n";
            return 1;
        }

        size_t vertex_count, triangle_count;
        int dimension, num_vertex_attributes, num_triangle_attributes, nodes_per_triangle;
        bool boundary_marker;

        read_node_header(vertex_count, dimension, num_vertex_attributes, boundary_marker, input_node);
        read_ele_header(triangle_count, nodes_per_triangle, num_triangle_attributes, input_ele);

        write_graph_header(vertex_count, triangle_count, output);
        convert_node_to_graph(input_node, output, vertex_count, boundary_marker);
        convert_ele_to_graph(input_ele, output, triangle_count);
    } else {
        print_help();
    }
}
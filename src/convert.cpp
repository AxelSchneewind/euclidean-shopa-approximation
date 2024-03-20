
#include <fstream>
#include <iostream>
#include <iomanip>

static constexpr int coordinate_precision = 20;

/*
 * Reads from the given input stream (containing only x and y coordinates) and converts to the .node format.
 * Assumes that all vertices are boundary vertices.
 *
 * specification for the .node format:
 *    First line:      <# of vertices> <dimension (must be 2)> <# of attributes> <# of boundary markers (0 or 1)>
 *    Remaining lines: <vertex #> <x> <y> [attributes] [boundary marker]
 */
void convert_vertices(std::ifstream& input, std::ofstream& output, size_t count) {
    int dimension = 2;
    int attributes = 0;
    int boundary_markers = 1;
    output << count << ' ' << dimension << ' ' << attributes << ' ' << boundary_markers << '\n';

    output << std::setprecision(coordinate_precision);
    for (size_t i = 0; i < count; ++i) {
        double x, y;
        input >> x >> y;
        output << i << ' ' << x << ' ' << y << " 1" << '\n';
    }
}

/*
 * Reads from the given input stream (containing 3 vertex ids per line) and converts to the .ele format.
 *
 * specification for the .ele format:
 *    First line: <# of triangles> <nodes per triangle> <# of attributes>
 *    Remaining lines: <triangle #> <node> <node> <node> ... [attributes]
 */
void convert_triangles(std::ifstream& input, std::ofstream& output, size_t count) {
    int nodes_per_face = 3;
    int attributes = 0;
    output << count << ' ' << nodes_per_face << ' ' << attributes << '\n';

    for (size_t i = 0; i < count; ++i) {
        size_t id1, id2, id3;
        input >> id1 >> id2 >> id3;
        output << i << ' ' << id1 << ' ' << id2 << ' ' << id3 << '\n';
    }
}



int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cout << "usage: convert path/to/input.graph path/to/output.node path/to/output.ele\n";
        return 1;
    }

    std::ifstream input(argv[1]);
    std::ofstream output_node(argv[2]);
    std::ofstream output_ele(argv[3]);

    if (input.bad() || output_node.bad() || output_ele.bad()) {
        std::cout << "some file cannot be written to or read from\n";
        return 1;
    }

    size_t vertex_count, triangle_count;
    input >> vertex_count >> triangle_count;

    convert_vertices(input, output_node, vertex_count);
    convert_triangles(input, output_node, triangle_count);
}
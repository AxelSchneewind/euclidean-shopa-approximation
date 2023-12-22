#include "graph/base_types_impl.h"
#include "graph/geometry_impl.h"
#include "file-io/formatters_impl.h"
#include "file-io/file_io_impl.h"

#include <fstream>


int main(int argc, const char* argv[]) {
    std::cout << "Reading graph from " << argv[1] << "..." << std::flush;
    // Graph graph = Graph::read_graph_file(argv[1], 0.0);
    std::ifstream input(argv[1]);
    long node_count, triangle_count;
    input >> node_count >> triangle_count;

    std::cout << " has " << node_count << " nodes and " << triangle_count << " faces. " << std::flush;

    std::vector<coordinate_t> nodes(node_count);
    file_io::read_nodes<coordinate_t, stream_encoders::encode_text>(input, nodes);

    std::vector<std::array<unsigned long, 3>> faces(triangle_count);
    file_io::read_triangles<unsigned long, stream_encoders::encode_text>(input, faces);

    std::cout << "\n, done" << std::endl;

    Projection projection = Projection::NONE;
    std::string projection_name(argv[3]);
    if (projection_name == "xy_to_lat_long")
        projection = Projection::GB_TO_WGS84;
    if (projection_name == "lat_long_to_xy")
        projection = Projection::WGS84_TO_GB;

    std::cout << "projecting with " << (int)projection << "..." << std::flush;
    // graph.project(projection);
    for(auto& node : nodes)
        project_coordinate(node, projection);

    std::cout << "\b\b\b, done" << std::endl;

    std::cout << "writing to " << argv[2] << "..." << std::flush;
    // graph.write_graph_file(argv[2]);
    std::ofstream output(argv[2]);
    output << node_count << std::endl;
    output << triangle_count << std::endl;
    file_io::write_nodes<coordinate_t, stream_encoders::encode_text>(output, nodes);
    file_io::write_triangles<unsigned long, stream_encoders::encode_text>(output, faces);
    std::cout << "\b\b\b, done" << std::endl;

    output.close();
}
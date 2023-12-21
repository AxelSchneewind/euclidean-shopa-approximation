
#include "interface/Graph.h"

#include "graph/geometry_impl.h"
#include "file-io/formatters_impl.h"
#include "file-io/fmi_file_io_impl.h"
#include "file-io/triangulation_file_io_impl.h"

#include "routing_impl.h"


int main(int argc, const char* argv[]) {
    std::cout << "Reading graph from " << argv[1] << "..." << std::flush;
    // Graph graph = Graph::read_graph_file(argv[1], 0.0);
    std::ifstream input(argv[1]);
    long node_count, triangle_count;
    input >> node_count >> triangle_count;

    auto nodes = fmi_file_io::read_nodes<node_t, stream_encoders::encode_text>(input, node_count);
    std::vector<std::array<node_id_t, 3>> faces;

    typename unidirectional_adjacency_list<node_id_t, float>::adjacency_list_builder triangles = triangulation_file_io::read_triangles<node_t, node_id_t, float, stream_encoders::encode_text>(input, nodes, triangle_count, faces);
    std::cout << "\b\b\b, done" << std::endl;

    Projection projection = Projection::NONE;
    std::string projection_name(argv[3]);
    if (projection_name == "xy_to_lat_long")
        projection = Projection::GB_TO_WGS84;
    if (projection_name == "lat_long_to_xy")
        projection = Projection::WGS84_TO_GB;

    std::cout << "projecting with " << (int)projection << "..." << std::flush;
    // graph.project(projection);
    for(auto& node : nodes)
        project_coordinate(node.coordinates, projection);

    std::cout << "\b\b\b, done" << std::endl;

    std::cout << "writing to " << argv[2] << "..." << std::flush;
    // graph.write_graph_file(argv[2]);
    std::ofstream output(argv[2]);
    fmi_file_io::write_nodes<node_t, stream_encoders::encode_text>(output, nodes);
    triangulation_file_io::write_triangles<node_id_t, stream_encoders::encode_text>(output, faces);
    std::cout << "\b\b\b, done" << std::endl;
}
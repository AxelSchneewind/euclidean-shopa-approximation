#include "file-io/file_io_impl.h"
#include "graph/geometry_impl.h"

#include <string>
#include <fstream>



int main(int argc, const char* argv[]) {
    if (argc < 3) {
        std::cout << "usage " << argv[0] << " path/to/graph-file path/to/result [xy_to_latlon|latlon_to_xy]\n";
        return 1;
    }

    std::string graph_file(argv[1]);
    std::string output_file(argv[2]);

    // select projection
    Projection projection = Projection::NONE;
    std::string projection_name(argv[3]);
    if (projection_name == "xy_to_latlon")
        projection = Projection::GB_TO_WGS84;
    if (projection_name == "latlon_to_xy")
        projection = Projection::WGS84_TO_GB;
    std::cout << "projecting with " << (int)projection << "...";

    // read nodes
    std::size_t node_count, other_count;

    std::vector<node_t> nodes(node_count);
    std::string rem;
    {
        std::ifstream input(graph_file);

        input >> node_count;
        input >> other_count;

        file_io::read_nodes<node_t>(input, nodes);
        rem = std::string(std::istreambuf_iterator<char>(input), {});
    }

    for (auto& node : nodes) {
        project_coordinate(node.coordinates, projection);
    }

    // write projected graph file
    {
        std::ofstream output(output_file);
        output << node_count << '\n' << other_count << '\n';
        file_io::write_nodes<node_t>(output, nodes);
        output << rem;
    }
}
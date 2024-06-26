#include "cli/cmdline_find_nodes.h"
#include "interface/Client.h"
#include "file-io/file_io_impl.h"
#include "graph/geometry_impl.h"

#include "triangulation/node_properties.h"
#include "graph/node_properties.h"

struct triangle_node_properties {
    bool connected:1;
    bool is_in_box:1;
    bool is_boundary_node:1;
    int out_degree;
    int in_degree;
};

int main(int argc, char** argv) {
    gengetopt_args_info args;
    int ret = cmdline_parser(argc, argv, &args);
    if (ret != 0 || args.help_given) {
        cmdline_parser_print_help();
        return 0;
    }

    std::string graph_file(args.graph_file_arg);
    std::ifstream input(graph_file);
    if (input.bad()) {
        std::cerr << "file cannot be read from\n";
        return 1;
    }

    // set file to write to
    std::streambuf * buf;
    std::ofstream of;
    if(args.output_file_given) {
        of.open(args.output_file_arg);
        buf = of.rdbuf();
    } else {
        buf = std::cout.rdbuf();
    }
    std::ostream output(buf);
    if (output.bad()) {
        std::cerr << "file cannot be written to\n";
        return 1;
    }

    // bounding box
    coordinate_t bottom_left{
        args.minY_given ? args.minY_arg : -std::numeric_limits<double>::infinity(),
        args.minX_given ? args.minX_arg : -std::numeric_limits<double>::infinity()
    };
    coordinate_t top_right{
            args.maxY_given ? args.maxY_arg : std::numeric_limits<double>::infinity(),
            args.maxX_given ? args.maxX_arg : std::numeric_limits<double>::infinity()
    };

    if (graph_file.ends_with(".graph")) {
        std::size_t node_count, face_count;
        input >> node_count;
        input >> face_count;

        std::vector<node_t> nodes(node_count);
        file_io::read_nodes<node_t>(input, nodes);

        std::vector<std::array<node_id_t, 3>> triangles(face_count);
        file_io::read_triangles<node_id_t>(input, triangles);

        // find nodes matching filter
        std::vector<triangle_node_properties> properties(node_count,  {false, false, false, 0, 0});
        compute_is_in_box(nodes, properties, bottom_left, top_right);
        compute_degrees(nodes, triangles, properties);
        compute_boundary_node(nodes, triangles, properties);

        if (args.pick_random_given) {
            // output some random node ids matching the criteria
            std::srand(args.seed_arg);
            for (int i = 0; i < args.pick_random_arg; i++) {
                int id = std::rand() % node_count;
                while (!((!args.boundary_nodes_flag || properties[id].is_boundary_node) && properties[id].is_in_box)){
                    id = std::rand() % node_count;
                }
                output << id << '\n';
            }
            output << std::flush;
        } else {
            // output all node ids matching the criteria
            for (int i = 0; i < args.pick_random_arg; i++) {
                if ((!args.boundary_nodes_flag || properties[i].is_boundary_node) && properties[i].is_in_box)
                    output << i << '\n';
            }
        }
    }
}
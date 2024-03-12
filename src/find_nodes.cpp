#include "cli/cmdline_find_nodes.h"
#include "interface/Client.h"
#include "file-io/file_io_impl.h"
#include "graph/geometry_impl.h"


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


    std::streambuf * buf;
    std::ofstream of;
    if(args.output_file_given) {
        of.open(args.output_file_arg);
        buf = of.rdbuf();
    } else {
        buf = std::cout.rdbuf();
    }

    std::ostream output(buf);

    coordinate_t bottom_left{
        args.minY_given ? args.minY_arg : -std::numeric_limits<double>::infinity(),
        args.minX_given ? args.minX_arg : -std::numeric_limits<double>::infinity()
    };
    coordinate_t top_right{
            args.maxY_given ? args.maxY_arg : std::numeric_limits<double>::infinity(),
            args.maxX_given ? args.maxX_arg : std::numeric_limits<double>::infinity()};

    if (graph_file.ends_with(".graph")) {
        std::size_t node_count, face_count;
        input >> node_count;
        input >> face_count;

        std::vector<node_t> nodes(node_count);
        file_io::read_nodes<node_t>(input, { nodes.begin(), nodes.end() });

        std::vector<std::array<node_id_t, 3>> triangles(face_count);
        file_io::read_triangles<node_id_t>(input, {triangles.begin(), triangles.end()});

        // find nodes matching filter
        std::vector<triangle_node_properties> properties(node_count);
        for (size_t i = 0; i < node_count; ++i) {
            properties[i].is_in_box = is_in_rectangle(nodes[i].coordinates, bottom_left, top_right);
        }

        // mark nodes as connected and increment out and in degree
        for (size_t t = 0; t < face_count; ++t) {
            auto&& triangle = triangles[t];
            properties[triangle[0]].connected = true;
            properties[triangle[1]].connected = true;
            properties[triangle[2]].connected = true;

            properties[triangle[0]].out_degree++;
            properties[triangle[1]].out_degree++;
            properties[triangle[2]].out_degree++;
            properties[triangle[0]].in_degree++;
            properties[triangle[1]].in_degree++;
            properties[triangle[2]].in_degree++;
        }

        // count faces for each edge
        std::unordered_map<long, char> adjacent_face_count;
        for (size_t t = 0; t < face_count; ++t) {
            auto&& triangle = triangles[t];

            adjacent_face_count[((long)triangle[0] << 32) + triangle[1]]++;
            adjacent_face_count[((long)triangle[1] << 32) + triangle[2]]++;
            adjacent_face_count[((long)triangle[2] << 32) + triangle[0]]++;
            adjacent_face_count[((long)triangle[0] << 32) + triangle[2]]++;
            adjacent_face_count[((long)triangle[1] << 32) + triangle[0]]++;
            adjacent_face_count[((long)triangle[2] << 32) + triangle[1]]++;
        }

        // if only one face adjacent to an edge, mark nodes as boundary nodes
        for (auto&& entry : adjacent_face_count) {
            assert(entry.second == 1 || entry.second == 2);
            if (entry.second == 1) {
                unsigned int node1 = entry.first >> 32;
                unsigned int node2 = ((entry.first << 32) >> 32);
                properties[node1].is_boundary_node = true;
                properties[node2].is_boundary_node = true;
            }
        }

        if (args.pick_random_given) {
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
            for (int i = 0; i < args.pick_random_arg; i++) {
                if ((!args.boundary_nodes_flag || properties[i].is_boundary_node))
                    output << i << '\n';
            }
        }
    }
}
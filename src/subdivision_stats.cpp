#include "interface/Client.h"
#include "routing_impl.h"
#include "triangulation/subdivision_table.h"
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>


// just to have the sizes somewhere and seen when it changes
static_assert(steiner_graph::SIZE_PER_NODE == 20);
static_assert(steiner_graph::SIZE_PER_EDGE == 56);

static_assert(std_graph_t::SIZE_PER_NODE == 20);
static_assert(std_graph_t::SIZE_PER_EDGE == 20);



int
main(int argc, char const *argv[]) {
    float epsilon = 0.50F;
    std::string mode("table");
    std::string graph_path;

    if (argc > 2) {
        mode = std::string(argv[1]);
        graph_path = std::string(argv[2]);
    } else {
        std::cout << "mode (table, graph): " << std::flush;
        std::cin >> mode;
        if (mode != "table" && mode != "graph")
            return 1;
        std::cout << "graph file: " << std::flush;
        std::cin >> graph_path;
    }

    if (argc > 3)
	    epsilon = std::stof(argv[3]);
    else {
    	std::cout << "epsilon: " << std::flush;
    	std::cin >> epsilon;
    }

    const double min_inner_angle = M_PI / 180; // 1º
    const double min_r_value = std::sin(min_inner_angle) * (1/(1 + std::sin(min_inner_angle) / std::sin(M_PI - min_inner_angle)));
    auto table = subdivision_table::precompute(epsilon, min_r_value);

    if (mode == "table") {
        std::cout << "epsilon,angle,number of points,points\n";
        int index = 0;
        for (auto triangle_class: table) {
            std::cout << epsilon << "," << std::setw(10) << subdivision_table::class_angle(index) << ","
                      << triangle_class.node_positions.size() << ",";

            for (auto point: triangle_class.node_positions) {
                std::cout << point << " ";
            }
            std::cout << '\n';

            index++;
        }
        std::cout << std::flush;
    } else {
        std::ifstream input(graph_path);
        auto graph = triangulation_file_io::read_steiner(input, epsilon);
        input.close();
        std::cout << "graph,epsilon,stored node count,stored edge count,stored boundary edge count,face count,node count,edge count";
        std::cout << "\n"<< graph_path
                  << ',' << epsilon
                  << ',' << graph.base_graph().node_count()
                  << ',' << graph.base_graph().edge_count()
                  << ',' << graph.base_polyhedron().boundary_edge_count()
                  << ',' << graph.base_polyhedron().face_count()
                  << ',' << graph.node_count()
                  << ',' << graph.edge_count() / 2 << '\n';
    }
}

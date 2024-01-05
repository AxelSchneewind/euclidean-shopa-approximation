#include "interface/Client.h"
#include "routing_impl.h"
#include "triangulation/subdivision_table.h"
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>


// just to have the sizes somewhere and see when they change
static_assert(steiner_graph::SIZE_PER_NODE == 20);
static_assert(steiner_graph::SIZE_PER_EDGE == 52);

static_assert(std_graph_t::SIZE_PER_NODE == 20);
static_assert(std_graph_t::SIZE_PER_EDGE == 20);



int
main(int argc, char const *argv[]) {
    double epsilon = 0.50F;
    std::string mode("table");
    std::string graph_path;

    if (argc > 2) {
        mode = std::string(argv[1]);
        graph_path = std::string(argv[2]);
    } else {
        std::cout << "mode (table, graph): " << std::flush;
        std::cin >> mode;
        std::cout << "graph file: " << std::flush;
        std::cin >> graph_path;
    }

    // accept fractions as input for epsilon
    std::string epsilon_string;
    if (argc > 3)
	    epsilon_string = argv[3];
    else {
    	std::cout << "epsilon: " << std::flush;
    	std::cin >> epsilon_string;
    }

    if (epsilon_string.find('/') != std::string::npos) {
        std::string numerator(epsilon_string, 0, epsilon_string.find('/'));
        std::string denominator(epsilon_string, epsilon_string.find('/') + 1);
        epsilon = std::stod(numerator) / std::stod(denominator);
    } else {
        epsilon = std::stod(epsilon_string);
    }

    // whether to print csv header or not
    bool header = false;
    if (argc > 4)
        header = true;

    const double min_inner_angle = M_PI / 720; // 1/4º
    const double min_r_value = std::sin(min_inner_angle) * (1/ (1 + std::sin(min_inner_angle) / std::sin(M_PI - min_inner_angle)));
    auto table = subdivision_table::precompute(epsilon, min_r_value);

    if (mode == "table") {
        if (header)
            std::cout << "epsilon,angle,number of points,points\n";
        int index = 0;
        for (auto const& triangle_class: table) {
            std::cout << epsilon << "," << std::setw(10) << subdivision_table::class_angle(index) << ","
                      << triangle_class.node_positions.size() << ",";

            for (auto point: triangle_class.node_positions) {
                std::cout << point << " ";
            }
            std::cout << '\n';

            index++;
        }
        std::cout << std::flush;
    } else if (mode == "graph") {
        std::string graph_prefix(graph_path, graph_path.find_last_of('/') + 1);
        std::string graph_filename(graph_path, graph_path.find_last_of('/') + 1);
        std::string graph_name(graph_filename, 0, graph_filename.find_last_of('.'));

        std::ifstream input(graph_path);
        auto graph = triangulation_file_io::read_steiner(input, epsilon);
        input.close();
        if (header)
            std::cout << "graph,epsilon,stored node count,stored edge count,stored boundary edge count,face count,node count,edge count\n";
        std::cout << graph_name
                  << ',' << epsilon_string
                  << ',' << graph.base_graph().node_count()
                  << ',' << graph.base_graph().edge_count()
                  << ',' << graph.base_polyhedron().boundary_edge_count()
                  << ',' << graph.base_polyhedron().face_count()
                  << ',' << graph.node_count()
                  << ',' << graph.edge_count() / 2 << '\n';
    } else if (mode == "angles") {
        std::ifstream input(graph_path);

        constexpr int bin_count = 180;
        constexpr double step_size = M_PI / (bin_count - 1);
        double min_angle = M_PI;
        double max_angle = 0.0;
        std::vector<std::size_t> angle_count(bin_count, 0);

        std::size_t node_count, triangle_count;
        input >> node_count >> triangle_count;

        std::vector<coordinate_t> coordinates(node_count);
        file_io::read_nodes<coordinate_t>(input, {coordinates.begin(), coordinates.end()});

        std::vector<std::array<unsigned long, 3>> triangles(triangle_count);
        file_io::read_triangles<unsigned long>(input, {triangles.begin(), triangles.end()});

        for (auto const &triangle: triangles) {
            coordinate_t c0, c1, c2;
            c0 = coordinates[triangle[0]];
            c1 = coordinates[triangle[1]];
            c2 = coordinates[triangle[2]];

            double alpha, beta, gamma;
            alpha = angle(c0, c1, c0, c2);
            beta = angle(c1, c0, c1, c2);
            gamma = angle(c2, c0, c2, c1);

            // angle has to be positive
            alpha = std::fabs(alpha);
            beta = std::fabs(beta);
            gamma = std::fabs(gamma);
            assert(alpha >= 0 && beta >= 0 && gamma >= 0);

            if (alpha >= M_PI)
                alpha = 2 * M_PI - alpha;
            if (beta >= M_PI)
                beta = 2 * M_PI - beta;
            if (gamma >= M_PI)
                gamma = 2 * M_PI - gamma;
            assert(alpha >= 0 && beta >= 0 && gamma >= 0);

            min_angle = std::min(min_angle, alpha);
            min_angle = std::min(min_angle, beta);
            min_angle = std::min(min_angle, gamma);
            max_angle = std::max(max_angle, alpha);
            max_angle = std::max(max_angle, beta);
            max_angle = std::max(max_angle, gamma);

            int index_alpha, index_beta, index_gamma;
            index_alpha = std::floor(alpha / step_size);
            index_beta = std::floor(beta / step_size);
            index_gamma = std::floor(gamma / step_size);
            assert(index_alpha >= 0 && index_beta >= 0 && index_gamma >= 0);
            assert(index_alpha < angle_count.size() && index_beta < angle_count.size() &&
                   index_gamma < angle_count.size());

            angle_count[index_alpha]++;
            angle_count[index_beta]++;
            angle_count[index_gamma]++;
        }

        input.close();

        if (header)
            std::cout << "angle,count\n";

        for (int index = 0; index < angle_count.size(); ++index) {
            std::cout << (index * step_size)
                      << ',' << angle_count[index] << '\n';
        }
        std::cout << std::flush;
    } else if (mode == "node count per edge" || mode == "npe") {
        std::ifstream input(graph_path);
        auto graph = triangulation_file_io::read_steiner(input, epsilon);

        if (header)
            std::cout << "edge,nodes\n";

        for (int e = 0; e < graph.base_graph().edge_count(); ++e) {
            auto &&steiner_info = graph.steiner_info(e);
            std::cout << e << ',' << steiner_info.node_count << '\n';
        }
        std::cout << std::flush;
    } else if (mode == "radii") {
        std::ifstream input(graph_path);
        auto graph = triangulation_file_io::read_steiner(input, epsilon);

        if (header)
            std::cout << "edge,r1,r2\n";

        for (int e = 0; e < graph.base_graph().edge_count(); ++e) {
            auto &&steiner_info = graph.steiner_info(e);
            std::cout << e << ',' << steiner_info.r_first << ',' << steiner_info.r_second << '\n';
        }
        std::cout << std::flush;
    }
}

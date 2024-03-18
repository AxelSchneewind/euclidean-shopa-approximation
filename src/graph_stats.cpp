#include "interface/Client.h"
#include "routing_impl.h"
#include "triangulation/subdivision_table.h"

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "cli/cmdline_graph_stats.h"


// just to have the sizes somewhere and see when they change
static_assert(steiner_graph::SIZE_PER_NODE == 24);
static_assert(steiner_graph::SIZE_PER_EDGE == 64);

static_assert(std_graph_t::SIZE_PER_NODE == 24);
static_assert(std_graph_t::SIZE_PER_EDGE == 40);


double parse_float_or_fraction(std::string const &epsilon_string) {
    double epsilon;
    if (epsilon_string.find('/') != std::string::npos) {
        std::string numerator(epsilon_string, 0, epsilon_string.find('/'));
        std::string denominator(epsilon_string, epsilon_string.find('/') + 1);
        epsilon = std::stod(numerator) / std::stod(denominator);
    } else {
        epsilon = std::stod(epsilon_string);
    }
    return epsilon;
}

std::string parse_graph_name(std::string const &graph_path) {
    std::string graph_prefix(graph_path, graph_path.find_last_of('/') + 1);
    std::string graph_filename(graph_path, graph_path.find_last_of('/') + 1);
    std::string graph_name(graph_filename, 0, graph_filename.find_last_of('.'));
    return graph_filename;
}


template<enum_mode Mode>
void show_info(gengetopt_args_info const &args);


template<>
void show_info<mode_arg_steiner_points_by_angle>(gengetopt_args_info const &args) {
    if (!args.no_header_flag)
        std::cout << "epsilon,angle,number of points,points\n";
    int index = 0;

    const double min_inner_angle = M_PI / 720; // 1/4º
    const double min_r_value = std::sin(min_inner_angle) *
                               (1 / (1 + std::sin(min_inner_angle) / std::sin(M_PI - min_inner_angle)));
    auto const table = subdivision_table::precompute(parse_float_or_fraction(args.epsilon_arg), min_r_value);

    for (auto const &triangle_class: table) {
        std::cout << parse_float_or_fraction(args.epsilon_arg) << "," << std::setw(10)
                  << subdivision_table::class_angle(index) << ","
                  << triangle_class.node_positions.size() << ",";

        for (auto const &point: triangle_class.node_positions) {
            std::cout << point << " ";
        }
        std::cout << '\n';

        index++;
    }
    std::cout << std::flush;
}

template<>
void show_info<mode_arg_steiner_graph_size>(gengetopt_args_info const &args) {
    std::ifstream input(args.graph_file_arg);
    auto graph = triangulation_file_io::read_steiner(input, parse_float_or_fraction(args.epsilon_arg));
    input.close();
    if (!args.no_header_flag)
        std::cout
                << "graph,epsilon,stored node count,stored edge count,stored boundary edge count,face count,node count,edge count\n";
    std::cout << parse_graph_name(args.graph_file_arg)
              << ',' << args.epsilon_arg
              << ',' << graph.base_graph().node_count()
              << ',' << graph.base_graph().edge_count()
              << ',' << graph.base_polyhedron().boundary_edge_count()
              << ',' << graph.base_polyhedron().face_count()
              << ',' << graph.node_count()
              << ',' << graph.edge_count() << '\n';
}

template<>
void show_info<mode_arg_points_per_edge>(gengetopt_args_info const &args) {
    std::ifstream input(args.graph_file_arg);
    auto graph = triangulation_file_io::read_steiner(input, parse_float_or_fraction(args.epsilon_arg));

    if (!args.no_header_flag)
        std::cout << "edge,nodes\n";

    for (size_t e = 0; e < graph.base_graph().edge_count(); ++e) {
        auto &&steiner_info = graph.steiner_info(e);
        std::cout << e << ',' << steiner_info.node_count << '\n';
    }
    std::cout << std::flush;
}

template<>
void show_info<mode_arg_node_radii>(gengetopt_args_info const &args) {
    std::ifstream input(args.graph_file_arg);
    auto graph = triangulation_file_io::read_steiner(input, parse_float_or_fraction(args.epsilon_arg));

    if (!args.no_header_flag)
        std::cout << "edge,r1,r2\n";

    for (size_t e = 0; e < graph.base_graph().edge_count(); ++e) {
        auto &&steiner_info = graph.steiner_info(e);
        std::cout << e << ',' << steiner_info.r_first << ',' << steiner_info.r_second << '\n';
    }
    std::cout << std::flush;
}


template<>
void show_info<mode_arg_inangle_distribution>(gengetopt_args_info const &args) {
    std::ifstream input(args.graph_file_arg);

    static constexpr int bin_count = 180;
    static constexpr double step_size = M_PI / (bin_count - 1);
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
        assert(alpha >= 0 && beta >= 0 && gamma >= 0);

        size_t index_alpha, index_beta, index_gamma;
        index_alpha = std::floor(alpha / step_size);
        index_beta = std::floor(beta / step_size);
        index_gamma = std::floor(gamma / step_size);
        assert(index_alpha < angle_count.size() && index_beta < angle_count.size() &&
               index_gamma < angle_count.size());

        angle_count[index_alpha]++;
        angle_count[index_beta]++;
        angle_count[index_gamma]++;
    }

    input.close();

    if (!args.no_header_flag)
        std::cout << "angle,count\n";

    for (size_t index = 0; index < angle_count.size(); ++index) {
        std::cout << (index * step_size)
                  << ',' << angle_count[index] << '\n';
    }
    std::cout << std::flush;
}

template<>
void show_info<mode_arg_bounding_box>(gengetopt_args_info const &args) {
    std::ifstream input(args.graph_file_arg);

    std::size_t node_count, triangle_count;
    input >> node_count >> triangle_count;

    std::vector<coordinate_t> coordinates(node_count);
    file_io::read_nodes<coordinate_t>(input, {coordinates.begin(), coordinates.end()});

    // get bounding box
    auto [min_latitude, max_latitude] = std::ranges::minmax(coordinates, std::ranges::less{},
                                                            &coordinate_t::latitude);
    auto [min_longitude, max_longitude] = std::ranges::minmax(coordinates, std::ranges::less{},
                                                              &coordinate_t::longitude);

    std::cout   << std::setprecision(20)
                << "latitude:  (" << min_latitude.latitude << ", " << max_latitude.latitude << "),\n"
                << "longitude: (" << min_longitude.longitude << ", " << max_longitude.longitude << ")\n";
}


int
main(int argc, char *argv[]) {
    gengetopt_args_info args;
    cmdline_parser(argc, static_cast<char **>(argv), &args);

    for (auto const& mode: std::span{args.mode_arg, args.mode_arg + args.mode_given})
        switch (mode) {
            case mode_arg_steiner_points_by_angle:
                show_info<mode_arg_steiner_points_by_angle>(args);
                break;
            case mode_arg_steiner_graph_size:
                show_info<mode_arg_steiner_graph_size>(args);
                break;
            case mode_arg_inangle_distribution:
                show_info<mode_arg_inangle_distribution>(args);
                break;
            case mode_arg_points_per_edge:
                show_info<mode_arg_points_per_edge>(args);
                break;
            case mode_arg_node_radii:
                show_info<mode_arg_node_radii>(args);
                break;
            case mode_arg_bounding_box:
                show_info<mode_arg_bounding_box>(args);
                break;
            case mode__NULL:
                cmdline_parser_print_help();
                break;
        }
}

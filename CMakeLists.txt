cmake_minimum_required(VERSION 3.22)
project(routing)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_COMPILER g++)

include_directories(include)

include(CPack)

set(CMAKE_CXX_FLAGS "-march=native -mtune=native -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "-g -D_GLIBCXX_DEBUG -Wall") # -fsanitize=address
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3")


# make file containing a version name
add_custom_target(version
        COMMAND echo -n "" > version.txt
        COMMAND git rev-parse --short HEAD >> version.txt
        # COMMAND date +%Y%m%d:%H%M >> version.txt
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "make version name"
        VERBATIM)

# doxygen
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # note the option ALL which allows to build the docs together with the application
    add_custom_target(doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT} -q
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
else (DOXYGEN_FOUND)
    message("Doxygen needs to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

set(VERSION 0.1)

# gengetopt
find_program(GENGETOPT gengetopt)
if (GENGETOPT)
    add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/src/cli/cmdline_graph_stats.c
            COMMAND ${GENGETOPT} -i src/cli/graph_stats.ggo -F cmdline_graph_stats --output-dir=src/cli  --set-version=${VERSION}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM)

    add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/src/cli/cmdline_route.c
            COMMAND ${GENGETOPT} -i src/cli/route.ggo -F cmdline_route --output-dir=src/cli  --set-version=${VERSION}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM)

    add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/src/cli/cmdline_one_to_all.c
            COMMAND ${GENGETOPT} -i src/cli/one-to-all.ggo -F cmdline_one_to_all --output-dir=src/cli  --set-version=${VERSION}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM)

    add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/src/cli/cmdline_find_nodes.c
            COMMAND ${GENGETOPT} -i src/cli/find_nodes.ggo -F cmdline_find_nodes --output-dir=src/cli  --set-version=${VERSION}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM)

    add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/src/cli/cmdline_prune_graph.c
            COMMAND ${GENGETOPT} -i src/cli/prune_graph.ggo -F cmdline_prune_graph --output-dir=src/cli --set-version=${VERSION}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM)

else (GENGETOPT)
    message("gengetopt needs to be installed to modify and generate the command line argument parser, falling back to precompiled (possibly out of date) command line parsing")
endif (GENGETOPT)


################################### BINARIES ###################################
# executables that may not be linked against librouting
add_executable(prune_graph src/prune_graph.cpp src/cli/cmdline_prune_graph.c)
add_executable(find_nodes src/find_nodes.cpp src/cli/cmdline_find_nodes.c)
add_executable(extract_boundary src/extract_boundary.cpp)

add_executable(graph_stats src/graph_stats.cpp src/cli/cmdline_graph_stats.c)


# compile implementation into one library file
add_library(routing src/interface/Client.cpp)
# target_compile_options(routing PUBLIC "-ffast-math")
link_libraries(routing)

# executables that will be linked against routing library
add_executable(project_graph src/project.cpp)
add_executable(make_explicit src/make_explicit.cpp)

add_executable(convert src/convert.cpp)
add_executable(make_gl src/make_gl.cpp)

add_executable(compute_shopa src/route.cpp src/cli/cmdline_route.c)
add_executable(compute_ota src/distance_one_to_all.cpp src/cli/cmdline_one_to_all.c)

################################# INSTALLATION #################################
# if access to this directory is missing, add a different one and append it to your PATH
set(CMAKE_INSTALL_PREFIX /opt/routing)
install(TARGETS compute_ota compute_shopa graph_stats make_gl extract_boundary make_explicit prune_graph project_graph convert find_nodes RUNTIME DESTINATION bin)

# make sure this library is in your PYTHONPATH for experiments to work
install(DIRECTORY model DESTINATION lib)


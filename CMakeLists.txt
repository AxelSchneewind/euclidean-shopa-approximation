cmake_minimum_required(VERSION 3.22)
project(routing)

set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_COMPILER /home/axel/software/aocc-compiler-4.2.0/bin/clang++)
set(CMAKE_CXX_COMPILER g++)

include_directories(include)


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

# gengetopt
find_program(GENGETOPT gengetopt)
if (GENGETOPT)
    add_custom_target(cmdline_graph_stats ALL
            COMMAND cp src/cli/graph_stats.ggo ${CMAKE_BINARY_DIR}/graph_stats.ggo
            COMMAND echo -n "version \"1.0\"" >> ${CMAKE_BINARY_DIR}/graph_stats.ggo
            COMMAND ${GENGETOPT} -i ${CMAKE_BINARY_DIR}/graph_stats.ggo -F cmdline_graph_stats --output-dir=src/cli  #--set-version=${VERSION}
            SOURCES src/cli/graph_stats.ggo
            DEPENDS version
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM)

    add_custom_target(cmdline_route ALL
            COMMAND cp src/cli/route.ggo ${CMAKE_BINARY_DIR}/route.ggo
            COMMAND echo -n "version \"1.0\"" >> ${CMAKE_BINARY_DIR}/route.ggo
            COMMAND ${GENGETOPT} -i ${CMAKE_BINARY_DIR}/route.ggo -F cmdline_route --output-dir=src/cli  #--set-version=${VERSION}
            SOURCES src/cli/route.ggo
            DEPENDS version
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM)

    add_custom_target(cmdline_find_nodes ALL
            COMMAND cp src/cli/find_nodes.ggo ${CMAKE_BINARY_DIR}/find_nodes.ggo
            COMMAND echo -n "version \"1.0\"" >> ${CMAKE_BINARY_DIR}/find_nodes.ggo
            COMMAND ${GENGETOPT} -i ${CMAKE_BINARY_DIR}/find_nodes.ggo -F cmdline_find_nodes --output-dir=src/cli  #--set-version=${VERSION}
            SOURCES src/cli/find_nodes.ggo
            DEPENDS version
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM)

    add_custom_target(cmdline_prune_graph ALL
            COMMAND cp src/cli/prune_graph.ggo ${CMAKE_BINARY_DIR}/prune_graph.ggo
            COMMAND echo -n "version \"1.0\"" >> ${CMAKE_BINARY_DIR}/prune_graph.ggo
            COMMAND ${GENGETOPT} -i ${CMAKE_BINARY_DIR}/prune_graph.ggo -F cmdline_prune_graph --output-dir=src/cli --set-version=${VERSION}
            SOURCES src/cli/prune_graph.ggo
            DEPENDS version
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM)

else (GENGETOPT)
    message("gengetopt needs to be installed to modify and generate the command line argument parser, falling back to precompiled (possibly out of date) command line parsing")
endif (GENGETOPT)


set(CMAKE_CXX_FLAGS "-march=native -mtune=native -Wall -Wextra -Wpedantic -fopenmp") # -Wextra
set(CMAKE_CXX_FLAGS_DEBUG "-g -fsanitize=address") # -D_GLIBCXX_DEBUG
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -Ofast")

# 
add_executable(prune_graph src/prune_graph.cpp src/cli/cmdline_prune_graph.c)
add_executable(find_nodes src/find_nodes.cpp src/cli/cmdline_find_nodes.c)

# compile implementation into one library file
add_library(routing src/interface/Client.cpp)
target_compile_options(routing PUBLIC "-ffast-math")
link_libraries(routing)

# executables that will be linked against routing library
add_executable(project_graph src/project.cpp)

add_executable(make_gl src/make_gl.cpp)
add_executable(make_pruned_gl src/make_pruned_gl.cpp)
add_executable(make_bin src/make_bin.cpp)

add_executable(route src/route.cpp src/cli/cmdline_route.c)

add_executable(distance_one_to_all src/distance_one_to_all.cpp)
add_executable(graph_stats src/graph_stats.cpp src/cli/cmdline_graph_stats.c)

if (GENGETOPT)
    add_dependencies(route cmdline_route)
    add_dependencies(prune_graph cmdline_prune_graph)
    add_dependencies(find_nodes cmdline_find_nodes)
    add_dependencies(graph_stats cmdline_graph_stats)
endif(GENGETOPT)

# installation
set(CMAKE_INSTALL_PREFIX /opt/routing)
install(TARGETS route graph_stats make_gl prune_graph project_graph find_nodes RUNTIME DESTINATION bin)

include(CPack)

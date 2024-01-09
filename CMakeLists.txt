cmake_minimum_required(VERSION 3.26)
project(routing)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_COMPILER g++)

include_directories(include)

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
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
else (DOXYGEN_FOUND)
    message("Doxygen needs to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

string(TIMESTAMP VERSION_TIMESTAMP %Y%m%d-%I%M)

# gengetopt
find_program(GENGETOPT gengetopt)
if (GENGETOPT)
    add_custom_target(cmdline_route ALL
            COMMAND ${GENGETOPT} -i src/cli/route.ggo -F cmdline_route --set-version=${VERSION_TIMESTAMP} --output-dir src/cli
            SOURCES src/cli/route.ggo
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    add_custom_target(cmdline_prune_graph ALL
            COMMAND ${GENGETOPT} -i src/cli/prune_graph.ggo -F cmdline_prune_graph --set-version=${VERSION_TIMESTAMP} --output-dir src/cli
            SOURCES src/cli/prune_graph.ggo
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
else (GENGETOPT)
    message("gengetopt needs to be installed to modify and generate the command line argument parser")
endif (GENGETOPT)


set(CMAKE_CXX_FLAGS "") # -Wall -Wextra
set(CMAKE_CXX_FLAGS_DEBUG "-g -fbounds-check") #-D_GLIBCXX_DEBUG

# performance options
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3 -march=native") # -ffast-math

add_executable(prune_graph src/prune_graph.cpp src/cli/cmdline_prune_graph.c)
add_dependencies(prune_graph cmdline_prune_graph)

# compile implementation into one library file
add_library(routing src/interface/Client.cpp)
link_libraries(routing)

# executables that will be linked against routing library
add_executable(project_graph src/project.cpp)

add_executable(make_gl src/make_gl.cpp)
add_executable(make_pruned_gl src/make_pruned_gl.cpp)
add_executable(make_bin src/make_bin.cpp)

add_executable(route src/route.cpp src/cli/cmdline_route.c)
add_dependencies(route cmdline_route)

add_executable(distance_one_to_all src/distance_one_to_all.cpp)
add_executable(subdivision_stats src/subdivision_stats.cpp)


# installation
set(CMAKE_INSTALL_PREFIX /opt/routing)
install(TARGETS route subdivision_stats make_gl prune_graph project_graph RUNTIME DESTINATION bin)


include(CPack)

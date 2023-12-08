cmake_minimum_required(VERSION 3.26)
project(routing)

set(CMAKE_CXX_STANDARD 23)
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
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target(doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

set(CMAKE_CXX_FLAGS "") # -Wall -Wextra
set(CMAKE_CXX_FLAGS_DEBUG "-g -fbounds-check")

# performance options
set(CMAKE_CXX_FLAGS_RELEASE " -DNDEBUG -O3 -march=native") # -ffast-math

# gengetopt target
configure_file(src/cli/route.ggo src/cli/cmdline_route.c @ONLY)
add_custom_target(cmdline ALL
        COMMAND gengetopt -i src/cli/route.ggo -F cmdline_route --output-dir src/cli
        SOURCES src/cli/route.ggo
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# compile implementation into one library file
add_library(routing src/routing.cpp)
link_libraries(routing)

add_executable(make_gl src/make_gl.cpp)
add_executable(make_pruned_gl src/make_pruned_gl.cpp)
add_executable(make_bin src/make_bin.cpp)

add_executable(route src/route.cpp src/cli/cmdline_route.c)
add_dependencies(route cmdline)

add_executable(distance_one_to_all src/distance_one_to_all.cpp)
add_executable(subdivision_stats src/subdivision_stats.cpp)


# installation
install(TARGETS route RUNTIME DESTINATION bin)

include(CPack)

cmake_minimum_required(VERSION 3.26)
project(routing)

set(CMAKE_CXX_STANDARD 23)

add_executable(routing main.cpp
        graph/bidirectional_adjacency_list.h
        graph/base_types.h
        routing/dijkstra_rules.h
)

add_executable(test_route_toy test_route_toy.cpp
        graph/bidirectional_adjacency_list.h
        graph/base_types.h
        routing/dijkstra_rules.h
)
add_executable(test_route_stgtregbz test_route_stgtregbz.cpp)

add_executable(test_read test_read.cpp)

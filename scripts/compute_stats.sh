#!/bin/bash

/home/axel/Dokumente/Studium/Semester_7/routing/cmake-build-release/subdivision_stats graph /home/axel/Studium/Semester_7/graphs/visibility_aegaeis/aegaeis10-ref-projected.graph 8.0 --header | tee -a graph_sizes.csv

# for epsilon in 4.0 2.0 1.0 '0.5' '0.25' '0.125' '0.0625' '0.03125' '0.015625' '0.0078125' '0.00390625' '0.001953125'
for epsilon in 4.0 2.0 1.0 '1/2' '1/4' '1/8' '1/16' '1/32' '1/64' '1/128' '1/256'
do
  echo computing for ε = $epsilon ...
  # compute graph sizes
  /home/axel/Dokumente/Studium/Semester_7/routing/cmake-build-release/subdivision_stats graph /home/axel/Studium/Semester_7/graphs/visibility_aegaeis/aegaeis10-ref-projected.graph $epsilon | tee -a graph_sizes.csv
done


/home/axel/Dokumente/Studium/Semester_7/routing/cmake-build-release/subdivision_stats graph /home/axel/Studium/Semester_7/graphs/visibility_aegaeis/aegaeis10-unref-projected.graph 8.0 --header | tee -a graph_sizes.csv
for epsilon in 4.0 2.0 1.0 '1/2' '1/4' '1/8' '1/16' '1/32' '1/64' '1/128' '1/256'
do
  echo computing for ε = $epsilon ...
  # compute graph sizes
  /home/axel/Dokumente/Studium/Semester_7/routing/cmake-build-release/subdivision_stats graph /home/axel/Studium/Semester_7/graphs/visibility_aegaeis/aegaeis10-unref-projected.graph $epsilon | tee -a graph_sizes.csv
done

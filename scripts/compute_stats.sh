#!/bin/bash


for epsilon in 2.56 1.28 0.64 0.32 0.16 0.08 0.04 0.02 0.01 0.005
do
  echo routing with epsilon = $epsilon
	# /home/axel/Dokumente/Studium/Semester_7/routing/cmake-build-release/subdivision_stats graph /home/axel/Studium/Semester_7/graphs/aegaeis/pruned.graph $epsilon >> graph_sizes.txt
  /home/axel/Dokumente/Studium/Semester_7/routing/cmake-build-release/route_interactive_steiner /home/axel/Studium/Semester_7/graphs/aegaeis/pruned.graph /home/axel/Studium/Semester_7/graphs/aegaeis/ $epsilon 2210302 235261 >> statistics.csv
done


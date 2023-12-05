#!/bin/bash


for epsilon in 4.0 2.0 1.0 0.5 0.25 0.125 0.0625 0.03125 0.015625 0.0078125 0.00390625
do
  echo computing for ε = $epsilon ...
  /home/axel/Dokumente/Studium/Semester_7/routing/cmake-build-release/route_interactive_steiner /home/axel/Studium/Semester_7/graphs/aegaeis/pruned.graph /home/axel/Studium/Semester_7/graphs/aegaeis/ $epsilon 2210303 235261 >> statistics.csv

done


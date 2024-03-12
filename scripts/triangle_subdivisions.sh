#!/bin/bash


graph_name=triangle
output_subdirectory=$graph_name

mkdir -p results/$graph_name/


for epsilon in 4.0 2.0 1.0 '0.5' '0.25' '0.125' '0.0625' '0.03125' '0.015625'
do
	epsilon_string=${epsilon//\./_}

	command="/opt/routing/bin/make_gl /opt/routing/graphs/$graph_name.graph results/$output_subdirectory/triangle_$epsilon_string.steiner.gl $epsilon 1 1"
	echo "$command"
	$command
done


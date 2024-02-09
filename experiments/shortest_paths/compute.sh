#/bin/bash

SCRIPTS=($(echo compute_*.sh))
for script in "${SCRIPTS[@]}"; do
	(sh -c "./$script" > out_$script.txt 2>&1 &)
done

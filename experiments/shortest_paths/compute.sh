#/bin/bash

SCRIPTS=($(echo compute_*.sh))
for script in "${SCRIPTS[@]}"; do
	echo "$script"
	echo ""
	# $(sh -c "./$script" > out_$script.txt 2>&1 &)
done

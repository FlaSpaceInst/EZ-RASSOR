#!/bin/bash
dot="$(cd "$(dirname "$0")"; pwd)"
DEMS="$dot/queued_dems/*"
#sudo chmod -R 777 /tmp/dem_results
for f in $DEMS
do
    name=$(basename $f ".tif")
    echo "Processing $name ..."
    echo "Modify to take in arguments: .tif, ..._out.txt, and /dem_to_txt/...origin.txt" 
    python3 "$dot/localMaxima.py" $f "$dot/dem_results/${name}_extr_out.txt" "$dot/dem_results/${name}_loc_max_out.txt"
done


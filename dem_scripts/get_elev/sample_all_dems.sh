#!/bin/bash
dot="$(cd "$(dirname "$0")"; pwd)"
DEMS="$dot/queued_dems/*"

for f in $DEMS
do
    name=$(basename $f ".tif")
    echo "Processing $name ..."
    python3 "$dot/localMaxima.py" $f "$dot/dem_results/${name}_extr_out.txt" "$dot/dem_results/${name}_loc_max_out.txt"
done

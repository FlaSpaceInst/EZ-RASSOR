#!/bin/bash
dot=/tmp
echo $dot
DEMS="$dot/queued_dems/*"

for f in $DEMS
do
    name=$(basename "$f" ".tif")
    echo "Processing $name ..."
    python3 "$dot/readDEM.py" "$f" "$dot/dem_results/${name}_extr_out.txt"
done

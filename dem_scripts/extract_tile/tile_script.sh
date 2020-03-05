#!/bin/bash

dot="$(cd "$(dirname "$0")"; pwd)"
DEMS="$dot/queued_dems/*"

# Give users the option to uniformal split each queued dem into k (nxn) tiles
read -p "All queued of same size? [y or n]" ans
all_size=0
if [ $ans = "y" ]
then
    read -p "What size? [129, 257, 513]" temp
    all_size=$temp
fi

for f in $DEMS
do
    name=$(basename $f ".tif")
    echo "Processing $name ..."

    # Check for uniform size, else ask for each one
    if [ $all_size -ne 0 ]
    then
        curr_size=$all_size
    else
        read -p "What size? [129, 257, 513]" ind_size
        curr_size=$ind_size
    fi

    python3 "$dot/tile.py" $f $all_size

    curr_size=0
done

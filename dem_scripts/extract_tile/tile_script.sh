#!/bin/sh

dot=/tmp
DEMS="$dot/queued_dems/*"

# Give users the option to uniformal split each queued dem into k (nxn) tiles
printf "All queued of same size? [y or n]\n"
read -r ans
all_size=0
if [ "$ans" = "y" ]
then
    printf "What size? [129, 257, 513]\n"
    read -r temp
    all_size=$temp
fi

for f in $DEMS
do
    name=$(basename "$f" ".tif")
    echo "Processing $name ..."

    # Check for uniform size, else ask for each one
    if [ "$all_size" -ne 0 ]
    then
        curr_size=$all_size
    else
        printf "What size? [129, 257, 513]\n"
        read -r ind_size
        curr_size=$ind_size
    fi

    python "$dot/tile.py" "$f" "$all_size"

    curr_size=0
done

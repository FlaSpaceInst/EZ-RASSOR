#!/bin/bash
dot="$(cd "$(dirname "$0")"; pwd)"
DEMS="$dot/queued_dems/*"
#sudo chmod -R 777 /tmp/dem_results
read -p "All queued of same size? [y or n]" ans
all_size=0
if [ $ans = "y" ]
then
    read -p "What size? [129, 257, 513]" temp
    all_size=$temp
fi 

curr_size=$all_size

for f in $DEMS
do
    name=$(basename $f ".tif")
    echo "Processing $name ..."
    
    if [ $all_size -ne 0 ]
    then
        curr_size=$all_size
    else
        read -p "What size? [129, 257, 513]" ind_size
        curr_size=$ind_size
    fi
    
    gdalwarp -ts $curr_size $curr_size $f "$dot/downsized_dems/${name}_resize.tif"
    gdal_translate -of JPEG -scale "$dot/downsized_dems/${name}_resize.tif" "$dot/converted_dems/${name}_converted.jpg"
    
    curr_size=0
done

#!/bin/sh

dot=/tmp
DEMS="$dot/queued_dems/*"

# Allow user to pick a size for all of them
printf "All queued of same size? [y or n]\n"
read -r ans
all_size=0
if [ "$ans" = "y" ]
then
    printf "What size? [129, 257, 513]"
    read -r temp
    all_size=$temp
fi

# Check what the size of the DEM is
check_curr_size() {
    
    x_y_dim=$(head -n 1 "$dot/check_file.txt")
    count=0
    x=0
    y=0
    for val in $x_y_dim; do
        if [ "$count" -eq 0 ]
        then
            x=$val
        else
            y=$val
        fi
        count=$(( count + 1 ))
    done

    change_size=0
    # If not square size, need to resize regardless
    if [ "$x" != "$y" ];
    then
        change_size=1
    else
        count_not_size=0
        for curr_size in 129 257 513; do
            if [ "$x" != "$curr_size" ]
            then
                count_not_size=$(( count_not_size + 1 ))
            fi
        done
        if [ "$count_not_size" -eq 3 ]
        then
            change_size=1
        fi
        # If it gets here and didn't execute the if block right above,
        # then it skips the resizing step since it's in the correct format
    fi
}

curr_size=$all_size

for f in $DEMS
do
    name=$(basename "$f" ".tif")
    echo "Processing $name ..."

    python "$dot/check_dem_size.py" "$f" > "$dot/check_file.txt"
    check_curr_size

    # If we need to resize the dem or not, but ultimately convert to jpg
    if [ "$change_size" -eq 1 ]
    then
        # If user specified a uniform size, else we ask for each file
        if [ "$all_size" -ne 0 ]
        then
            curr_size=$all_size
        else
            printf "What size? [129, 257, 513]\n"
            read -r ind_size
            curr_size=$ind_size
        fi

        gdalwarp -ts "$curr_size" "$curr_size" "$f" "$dot/downsized_dems/${name}_resize.tif"
        gdal_translate -of JPEG -scale "$dot/downsized_dems/${name}_resize.tif" "$dot/converted_dems/${name}_converted.jpg"
    else
        echo "Skipping DEM resizing"
        gdal_translate -of JPEG -scale "$f" "$dot/converted_dems/${name}_converted.jpg"
    fi

    curr_size=0
done

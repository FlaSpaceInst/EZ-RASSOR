#!/bin/bash
dot="$(cd "$(dirname "$0")"; pwd)"
DEMS="$dot/queued_dems/*"

gdalinfo --formats

for f in $DEMS
do
    # Obtains filename from path i.e. /home/username/hello.txt -> hello.txt
    name=$(basename -- $f)
    # Obtains the extension i.e. hello.txt -> txt
    extension="${name##*.}"
    # name becomes just the name, no extension i.e. hello
    name="${name%.*}"

    # Look only at the .lbl files
    if [ "$extension" = "lbl" ]
    then
        echo "Processing $name ..."
        # Convert .lbl to .tif
        gdal_translate $f "$dot/results/${name}.tif"
    fi
done

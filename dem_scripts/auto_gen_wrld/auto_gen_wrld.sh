#!/bin/bash
dot="$(cd "$(dirname "$0")"; pwd)"
DEMS="$dot/queue/*"
ez_dir=$(dirname $(dirname $dot))
for f in $DEMS
do
    name=$(basename $f ".jpg")
    echo "Processing $name ..."
    read -p "Please enter name for world:" temp
    mkdir "$dot/results/${temp}_pack"
    mkdir "$dot/results/${temp}_pack/$temp"
    mkdir "$dot/results/${temp}_pack/$temp/materials"
    python "$dot/generate_xmls.py" $f $temp "results/${temp}_pack/"
    cp "$dot/AS16-110-18026HR-512x512.jpg" "$dot/results/${temp}_pack/$temp/materials"
    cp "$f" "$dot/results/${temp}_pack/$temp/materials"
    cp -ar "$dot/results/${temp}_pack/$temp/" "$HOME/.gazebo/models/"
    cp "$dot/results/${temp}_pack/${temp}.world" "$ez_dir/packages/simulation/ezrassor_sim_gazebo/worlds/"
done

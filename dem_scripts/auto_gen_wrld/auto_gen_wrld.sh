#!/bin/bash

dot="$(cd "$(dirname "$0")"; pwd)"
DEMS="$dot/queue/*"
ez_dir=$(dirname $(dirname $dot))
rm -rf results/
mkdir results
for f in $DEMS
do
    name=$(basename $f ".jpg")
    echo "Processing $name ..."
    read -p "Please enter name for world:" temp
    if [[ -d "$HOME/.gazebo/models/${temp}" ]]
    then
    	rm -rf "$HOME/.gazebo/models/${temp}/"
    	if [[ -d "$HOME/.gazebo/paging/${name}" ]]
    	then
    		rm -rf "$HOME/.gazebo/paging/${name}"
    	fi
        if [[ -f "$ez_dir/packages/simulation/ezrassor_sim_gazebo/worlds/${temp}.world" ]]
        then
            rm "$ez_dir/packages/simulation/ezrassor_sim_gazebo/worlds/${temp}.world"
        fi
    fi
    if [[ -d "$dot/results/${temp}_pack" ]]
    then
    	rm -rf "$dot/results/${temp}_pack"
    fi
    mkdir "$dot/results/${temp}_pack"
    mkdir "$dot/results/${temp}_pack/$temp"
    mkdir "$dot/results/${temp}_pack/$temp/materials"
    cp -a "$dot/template_model_n_world/s_pole/." "$dot/results/${temp}_pack/$temp"
    cp "$dot/template_model_n_world/try_new_model.world" "$dot/results/${temp}_pack/${temp}.world"
	cp $f "$dot/results/${temp}_pack/$temp/materials/textures" 
	rm "$dot/results/${temp}_pack/$temp/materials/textures/LRO_PNG_TEST_257.jpg"
	python model_create.py "$dot/results/${temp}_pack/${temp}/model.sdf" $temp "${name}.jpg"
    python world_file_create.py "$dot/results/${temp}_pack/${temp}.world" $temp "${name}.jpg"
	cp -r "$dot/results/${temp}_pack/$temp/" "$HOME/.gazebo/models/"
    cp "$dot/results/${temp}_pack/${temp}.world" "$ez_dir/packages/simulation/ezrassor_sim_gazebo/worlds/"
done
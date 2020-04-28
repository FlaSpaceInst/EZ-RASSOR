#!/bin/bash

dot="$(cd "$(dirname "$0")"; pwd)"
DEMS="$dot/queue/*"
ez_dir=$(dirname $(dirname $dot))

for f in $DEMS
do
    name=$(basename $f ".jpg")
    echo "Processing $name ..."
    read -p "Please enter name for model: " temp
    echo "Model Name: ${temp}, World Name: ${temp}_world.world"

    # If a model already exists with the same name
    if [[ -d "$HOME/.gazebo/models/${temp}" ]]
    then

    	rm -rf "$HOME/.gazebo/models/${temp}/"
    	echo "Removed existing ${temp}/"

        # If a gazebo cache already exists for the used raster (i.e. DEM_FILE.jpg)
        if [[ -d "$HOME/.gazebo/paging/${name}" ]]
    	then
    		rm -rf "$HOME/.gazebo/paging/${name}"
            echo "Removed existing cache for ${name}"
    	fi

        # If a world already exists with the same name
        if [[ -f "$ez_dir/packages/simulation/ezrassor_sim_gazebo/worlds/${temp}_world.world" ]]
        then
            rm "$ez_dir/packages/simulation/ezrassor_sim_gazebo/worlds/${temp}_world.world"
            echo "Removed existing ${temp}_world.world from ../packages/simulation/ezrassor_sim_gazebo/worlds/"
        fi

    fi

    # If there already exists a result with the same name
    if [[ -d "$dot/results/${temp}_pack" ]]
    then
    	rm -rf "$dot/results/${temp}_pack"
        echo "Removed existing ${temp}_pack/ from results"
    fi

    # Get width and height from image, also set default range for heightmap
    dimmensions_w=$(identify -format '%w' $f)
    dimmensions_h=$(identify -format '%h' $f)
    squish_factor=25

    echo "File is of ${dimmensions_w} ${dimmensions_h}" 
    
    read -p "Would you like to specify a squish (highest - lowest elevation) factor? [y or n]" decide_squish
    
    if [ $decide_squish = "y" ]
    then
        read -p "Please enter squish factor: " user_squish
        squish_factor=$user_squish
    fi

    # Make folder structure and copy template files
    mkdir "$dot/results/${temp}_pack"
    cp -a "$dot/template_model_n_world/s_pole/." "$dot/results/${temp}_pack/$temp"
    cp "$dot/template_model_n_world/try_new_model.world" "$dot/results/${temp}_pack/${temp}_world.world"
	
    # Copy DEM_FILE.jpg over to textures
    cp $f "$dot/results/${temp}_pack/$temp/materials/textures" 
	
    # Remove original DEM_FILE.jpg from template model
    rm "$dot/results/${temp}_pack/$temp/materials/textures/LRO_PNG_TEST_257.jpg"
	
    echo "Update ${temp}_pack/$temp/model.sdf"
    python model_create.py "$dot/results/${temp}_pack/${temp}/model.sdf" $temp "${name}.jpg" $dimmensions_w $dimmensions_h $squish_factor
    echo "Update ${temp}_pack/${temp}_world.world"
    python world_file_create.py "$dot/results/${temp}_pack/${temp}_world.world" $temp "${name}.jpg" $dimmensions_w $dimmensions_h $squish_factor
	
    # Copy model and world to respective place
    cp -r "$dot/results/${temp}_pack/$temp/" "$HOME/.gazebo/models/"
    cp "$dot/results/${temp}_pack/${temp}_world.world" "$ez_dir/packages/simulation/ezrassor_sim_gazebo/worlds/"
done
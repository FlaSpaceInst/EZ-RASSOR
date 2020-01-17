README for mk_gaz_wrld and get_elev

*********************************** Pipeline ****************************************************
                    (Bulletpoints simply state what files CAN be used for) 
    
    mk_gaz_wrld -> (resized dem, jpg)
        jpg -> make gaz model -> make gaz world 
        resized dem -> get_elev
    get_elev -> (text file matrix data, text file local maxima coordinates : elevation)
        text file matrix data -> find selen coordinates of (x, y) pixel, elev at (x, y)
        text file local maxima (x,y) coordinates : elevation -> compare with observed horizon
    
************************************************************************************************

************************************* IMPORTANT *************************************************

    1. DO NOT PUSH ORIGINAL DEMS TO GITHUB, SINCE MOST DEMS ARE QUITE LARGE, YOUR COMMIT PROBABLY WILL BE REJECTED IF THE FILE IS OVER THE 100 MB AND FILES 50-100 MB WILL BE GIVEN A WARNING.
    2. AFTER RUNNING, PLEASE AT LEAST MAKE SURE TO EXECUTE RESET BEFORE RUNNING THE SCRIPT AGAIN.
    3. YOU MAY HAVE TO CHANGE THE PERMISSIONS FOR THE RESULTING FILES.
    4. PLEASE ONLY USE .tif FILES. IF YOU WANT TO USE OTHER DEM TYPES, REFER TO THE gdal DOCUMENTATION AS TO HOW TO MODIFY script_convert.sh IN mk_gaz_wrld AND POSSIBLY localMaxima.py OR testGdal.py IN get_elev.   

*************************************************************************************************

******************************** BUILDING AND EXECUTING ***************************************    

The only thing you need besides the files in this directory is Docker. After installing Docker, you should be able to run this script although you may need to run sudo depending on your access level with Docker.

The following programs that this script can run is mk_gaz_wrld and get_elev.
    
    mk_gaz_wrld - given a .tif file, it will downsize it and also produce a .jpg from the downsized .tif. The .jpg will allow you to put it in gazebo as a heightmap or you can put it in as a model so it persists after you restart your computer. The latter requires extra steps but the example s_pole directory contains the all that's need to make a model and the moon_test.world is the file that references the s_pole model.
    
    Overview of how to make gazebo world with results (Refer to included example or google for details):
    
    - Make a model and put in ~/.gazebo/models/
        - Name_of_model_dir/ 
            - model.config
            - model.sdf
            - materials/
    - World file which is moved to EZ-RASSOR/packages/simulation/ezrassor_sim_gazebo/worlds
        - ref model in uri and textures as "model://Name_of_model_dir/path_to_files"
        - add physics
    
    get_elev - given a .tif file, it will output a text file containing the original elevation data, as well as the decimal degree coordinate of the corner pixels, and it will also output a textfile containing the indices of the local maxima elevation points.
    
    If you select the option to run both programs, get_elev will be run with the downsized .tif file from mk_gaz_wrld.
    
To run the script, you sh run_programs.sh [KEYWORD], where [KEYWORD] is replaces with one of the following keywords.  

Keywords:
    
    reset - deletes files in queued_dems, downsized_dems, converted_dems, and dem_results
    populate - copy file from queue to queued_dems in mk_gaz_wrld
    clean - removes all docker containers, images, etc. THIS WILL REMOVE ALL DOCKER STUFF CREATED ON THE COMPUTER
    queue_reset - deletes files in the queue
    results_reset - deletes files in the results 
    run
        1 - copies files from queue then runs get_elev
        2 - runs mk_gaz_wrld
        3 - runs mk_gaz_wrld and then copies resized dem so that it can run get_elev
        
Output files are in respective folders, each program's output files are specified in pipeline
    - All files are also included in results folder
    
Example execution:
        
    So first you need to copy your files into the queue folder. Next, you run the populate command and then you can run the run command. After execution, you can run the reset the command and then rerun the prior commands in order. The queue_reset and results_reset commands are pretty self explainatory. The clean command is very important, although the script removes the containers from memory, the clean command is there to clean out all Docker things created ever on your computer. If you don't run this, you will have to manually remove the Docker images you don't want, if you wish to clean up the cache. Everytime you run this with different .tif files or of just a new combination of known .tif files, it will put them in Docker cache memory so it would be best to run the clean command before you shut off your computer if you're worried about memory consumption.


An example of what the results of mk_gaz_wrld could be used to make:

    Move the s_pole directory to ~/.gazebo/models and moon_test.world should already be in /packages/simulation/ezrassor_sim_gazebo/worlds. You now run the roslaunch as normal but simply add the flag to your command "world:=moon_test.world".

    Original DEM used as input for mk_gaz_wrld to create moon_test.world is from: https://astrogeology.usgs.gov/search/map/Moon/LRO/LOLA/Lunar_LRO_LOLA_Global_LDEM_118m_Mar2014 and it's the smaller file for south pole

************************************************************************************************** 


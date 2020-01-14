README for mk_gaz_wrld and get_elev
TODO is towards end of the file

*********************************** Pipeline ****************************************************
                    (Bulletpoints simply state what files CAN be used for) 
    
    mk_gaz_wrld -> (resized dem, jpg)
        jpg -> make gaz model -> make gaz world 
        resized dem -> get_elev
    get_elev -> (text file matrix data, text file local maxima coordinates : elevation)
        text file matrix data -> find selen coordinates of (x, y) pixel, elev at (x, y)
        text file local maxima (x,y) coordinates : elevation -> compare with observed horizon
    
************************************************************************************************

******************************** BUILDING AND EXECUTING ***************************************    
Original DEM is from: https://astrogeology.usgs.gov/search/map/Moon/LRO/LOLA/Lunar_LRO_LOLA_Global_LDEM_118m_Mar2014 and it's the smaller file for south pole

Please clean out the folders for the dems and resulting folders
    
Build the Dockefile.base with the tag "mybase":
    docker image build --tag mybase -f Dockerfile.base .
    
Move the DEMs (.tif) you want to sample, or put in format for gazebo, into the queued_dems folder

Build the Dockerfile.child with whatever tag you want:
     docker image build --tag child_img:ver_num -f Dockerfile.child .

    The version number isn't required but it helps in the case if you want to compare some functionalities. With each version, they cache upon a previous version or even the last build of the same version. If you change a command in a Dockerfile with previous versions, it will cache everything from the beginning of the file to the line that you changed, everything below it will be loaded into memory.
    
Run the container:
    sudo docker run --rm -it -rm -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)",target=/tmp,type=bind --name whatevername child_imag:ver_num

Output files are in respective folders, each program's output files are specified in pipeline

For loading the moon_test.world, move the s_pole directory to ~/.gazebo/models and then run the roslaunch as normal, get the world by "world:=moon_test"

*************************************************************************************************

************************************* IMPORTANT *************************************************

    1. DO NOT PUSH ORIGINAL DEMS TO GITHUB, SINCE MOST DEMS ARE QUITE LARGE, YOUR COMMIT PROBABLY WILL BE REJECTED IF THE FILE IS OVER THE 100 MB AND FILES 50-100 MB WILL BE GIVEN A WARNING.
    2. IF YOU WANT TO RUN ONE OF THE PROGRAMS MORE THAN ONCE AND OR PUT NEW DEMS IN THE queued_dems FOLDER, PLEASE BUILD THE CORRESPONDING IMAGE AGAIN AND THEN EXECUTE THE COMMAND TO RUN THE CONTAINER.
    3. IF YOU'RE DOING #2, IT'S BETTER TO EMPTY OUT THE FOLDERS BEFORE PUTTING IN NEW DEMS TO CUT BACK ON MEMORY NEEDED TO RUN THE CONTAINER SINCE IT EVERYTHING IN THE queued_dems WILL BE COPIED OVER REGARDLESS IF THERE ARE RESULTS ALREADY
        --------Might make a script to do this-----------
    4. TO CLEAN THE MEMORY MADE BY EITHER OF THESE PROGRAMS, YOU MUST REMOVE THE DOCKER IMAGES THAT ARE BUILT. IF YOU LEAVE THE IMAGES IN, IT HELPS TO CACHE EVERYTHING CUZ OTHERWISE, IT WILL HAVE TO INSTALL EVERYTHING AGAIN.    

************************************************************************************************* 



TODO/Ideas
- See how necessary is the .base Dockerfile    
- Maybe split dem into tiles
- Maybe move common .sh, folder, etc. to dem_scripts/
- Maybe sample local max and min for peaks and valleys
    - have altimeter
- Maybe script to download and remove dems
- Maybe script to clean out folders or move contents to separate folder
- Maybe script to actually pipeline
    - Option to either mk_gaz_wrld or get_elev or both
        - Order for both: mk_gaz_wrld -> get_elev   
- Check for mk_gaz_wrld formats of pds or isis
- Check Docker manifest
- Not too sure if can make script to make gazebo worlds with .jpg dems
    - It would have to generate model and put in ~/.gazebo/models/
        - Name_of_model_dir/ 
            - model.config
            - model.sdf
            - materials/
    - World file which is moved to EZ-RASSOR/packages/simulation/ezrassor_sim_gazebo/worlds
        - ref model in uri and textures as "model://Name_of_model_dir/path_to_files"
        - add physics

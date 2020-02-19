dem_scripts README

****************************** Pre-requisites *******************************************

Prior to running this script ensure you:
    - Have docker installed
    - Have one or multiple Digital Elevation Models (DEM) you wish to test
        - Place the DEM(s) in the dem_scripts/queue folder

************************ Important Notes *************************************************

Note about Github:
    DO NOT PUSH ORIGINAL DEMS TO GITHUB, SINCE MOST DEMS ARE QUITE LARGE, YOUR
    COMMIT PROBABLY WILL BE REJECTED IF THE FILE IS OVER THE 100 MB AND FILES
    50-100 MB WILL BE GIVEN A WARNING.

Note about Docker:
    The more you run this script, the more memory the docker images will take up
    since it appears that each build of an image is kept in like a linked list cache.
    It may also take sometime to load up a docker image since it needs to install
    dependencies before running a program. But once you have run one of programs before, it
    will cache some of the computation when you run it again. However, this may create
    another image, which is something to keep in mind if you're using docker for more
    than this application. If you want to keep your other docker stuff, you'll have to
    manually delete the images. Otherwise, you can simply run the clean command and it
    will wipe all docker containers, images, etc. Because the containers are set
    to remove themselves after execution, you just need to get rid of the images.

      Remove all docker images:
        docker rmi $(docker images -q)
      Remove a docker image by tag:
        docker rmi test1:ver_number

    If you would like to know why we're using docker and some details about how
    it works, see the section below labeled "Docker: Isolation"


***************************** Running the programs *************************************

NOTE: Please run the reset, queue_reset, and results_reset command first since
there is just a placeholder file in the otherwise empty folders

To run the script:
    sudo bash run_programs.sh WHATEVER_COMMAND

    NOTE: sudo is not necessary but you have to make sure that you're a part of
    the docker group permissions on your computer, see the link for details:
    https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user

Script commands:
    queue_reset - deletes all files from dem_scripts/queue folder
    results_reset - deletes all files from dem_scripts/results folder
    reset - deletes all files from each respective program's nested queued_dems
            and results folders but not dem_scripts/queue nor dem_scripts/results
    clean - purge your system of docker containers, images, etc.
    run

Running the programs:
    When you execute the script with the run command, you will be prompted with
    how many programs you would like to run, from 0 to 4. If in range and not 0,
    you will be prompted with what program would you like to run for each iteration.
    If you run more than 1 program, the output for each sequential execution will
    feed off each other. For example, if you say you want to run 2 programs,
    extract_tile and mk_gaz_wrld, once extract_tile is finished, the results
    will be queued into mk_gaz_wrld. The script allows you to order the programs
    however you want, just keep in mind that some permutations aren't efficient
    or just redundant. If you want to do all of them in a single run, the
    recommended order is 4->3->2->1.

    Programs:

    1 - get_elev:
            Given a DEM (.tif), it will output two text files:

                - DEM_FILE_NAME_extr_out.txt - consists of the decimal coordinates
                (lat, long) of each corner pixel, dimensions of the DEM, and all
                the elevation values

                - DEM_FILE_NAME_loc_max_out.txt - gets the local maxima values
                and the (row, col) where they occur within the DEM

    2 - mk_gaz_wrld:
            Given a DEM (.tif), it will output two files:

                - DEM_FILE_NAME_resized.tif - a downsized dem of a given size
                - DEM_FILE_NAME_converted.jpg - converted dem to jpg using _resized.tif

            You can use the .jpg to create a gazebo world, see the respectively
            named section below

    3 - extract_tile:
            Given a DEM (.tif), it will create DEM tiles (.tif) for each part of the DEM:

                - DEM_FILE_NAME_SOMEX_SOMEY.tif - extracted "tile" of a given size

            This functionality is included to keep the right scaling factor when
            trying to create a gazebo world from a DEM. DEMs are quite large and
            represent quite large areas, so to get it into the right format for
            gazebo, we have to downsize or compress it, which can make quite
            exaggerated and unrealistic terrain.

      4 - convert2tif:
              Given a PDS (.lbl + ((aux.xml + .jp2) or (.img))), converts to a
              DEM (.tif). Might work with ISIS files (.cub + .lbl) but not tested

              If you encounter errors such as:
                  band 1: IReadBlock failed at X offset 0, Y offset 0: Failed
                  to read scanline 0

                  or

                  0ERROR 1: Tile part length size inconsistent with stream length
              Either part of the file is corrupted or missing, try using a
              different pds or re-download it

IMPORTANT NOTE: Before executing run again, you might want to at least execute
the reset command otherwise it will execute the old jobs you sent to them as well
as the new ones in the queues.

Output:
    Output files can be found in each program's nested results folder and also the
    dem_scripts/results folder (fed all the output files of each program).
    NOTE: You may have to change the permissions on the files to edit them.



****************************** Making a Gazebo World ***********************************

After executing the mk_gaz_wrld, you can use the outputted jpg file to create
a gazebo world. Unfortunately, we don't have functionality to automate this
for you but it is possible for to create a script that does since world files
are essential xml.

Definitions:

    - Model:
        This is essential making the jpg into a persistent gazebo world object
        you call in the world file. This consists of multiple items, all within
        a model_name folder:
            - model.config
                References the .sdf file and has metadata
            - model.sdf
                Reference the path to the dem jpg (creates the geometry of
                the object) as well as to paths for normal and diffuse texturing
            - materials/
                Stores the jpg and all the other files used for textures or appearance
    - World:
        This is where you define the environment that the rover will load up into.

Quick overview:

    - Make a model and put in ~/.gazebo/models/
        - Name_of_model_dir/
            - model.config
            - model.sdf
            - materials/
    - World file which is moved to EZ-RASSOR/packages/simulation/ezrassor_sim_gazebo/worlds
        - ref model in uri and textures as "model://Name_of_model_dir/path_to_files"
        - add physics

Quick and Dirty Setup:
    The best way to understand is to look at the example and gazebo documentation.
    Or you can copy and tweak the model and world examples included in the current
    directory to create your own world. These instructions will make a world of lunar
    appearance and physics so adjust for your application.

    1. Rename the model folder
    2. In the .sdf, change the filename in the <heightmap><uri> tag to the
    name of the new dem jpg
    3. Also in the .sdf, change the base of the path "model://original_model/..."
    to "model://whatever_you_renamed_the_folder" for each reference to an image
    4. Remove the old dem jpg from your materials folder and add the new dem jpg to it
    5. Copy the model folder into your ~/.gazebo/models/
    6. Rename the world file to whatever you want
    6. In the world file, change the filenames and "model://name_of_model"
    to account for the new model name and the new dem jpg
    7. Move the world file into EZ-RASSOR/packages/simulation/ezrassor_sim_gazebo/worlds/
    8. You can now use this world by adding the flag "world:=whatever_you_named_the_world_without_the_extension"
    when launching the simulation

***************************** Docker: Isolation *********************************************

Because DEM readers aren't built into Ubuntu, we need to use either an application,
driver, or a library that can be used to read them in. In most applications and libraries,
they use a library called GDAL (https://gdal.org/) as the base for all their functionality.
GDAL is "a translator library for raster and vector geospatial data formats" of which
includes support for PDS (Nasa's Planetary Data System format) and GeoTiff (.tiff). The
problem is that GDAL conflicts with the dependencies for Gazebo so in order to do read DEMs
without breaking the environment to run the EZ-RASSOR simulation, we have to isolate it.

We have 3 options: VirtualMachine, Docker, or Anaconda. VMs are quite heavy since
we don't need a whole operating system, just a terminal. Anaconda is popular package
manager for python and can also provide isolated environments for python. The
main problems with it are that since it doesn't play well with ROS out of the box
and it can install excess packages (mostly data science) we don't need for our
application. Anaconda would be a great choice if you're already using python
for data science stuff but in our case, it's only for EZ-RASSOR. In hindsight,
there is the lighter version of Anaconda called Miniconda that you could use but
learning docker can be applied to more fields so using docker for only this application
isn't as bloatware-y as -conda stuff is to non-data scientists.

In terms of docker implementation, each program has there own docker image associated
with it. If you notice, there is a Dockerfile.base and a Dockerfile.child file rather
than the standard one Dockerfile per directory. The Dockerfile.base and entrypoint.sh
are just for setting up the program to run as a local user vs root. Even though that
a docker container is used for isolation, it's insecure to run as root.

Why to not run as root (there are other articles that mention this):
https://americanexpress.io/do-not-run-dockerized-applications-as-root/

Another thing that we do that is not standard, we use bind mounts to mount a host
directory inside a docker container. The most popular way to store data with Docker
is through volumes but isn't the ideal for our application. Volumes are usually the
recommended choice since they are more secure and a great way to transfer data from
container to container. Although we could store our data in volumes and pass it to
other containers created if a user runs more than one of the programs in the script,
we ultimately need to get the results back to the host, which isn't possible or at
least quite difficult to do. This makes bind mounts the best choice in this case.

Full comparison of storage formats in Docker:
https://docs.docker.com/storage/

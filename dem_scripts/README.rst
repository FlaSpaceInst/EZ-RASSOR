"""""""""""""""""
DEM Scripts
"""""""""""""""""

.. contents:: Table of Contents
   :depth: 3

===================
Pre-Requisites
===================

Prior to running this script, ensure you:
	- Have Docker installed
	- Have one or multiple Digital Elevation Models (DEM) you wish to test
		- Place the DEM(s) in the dem_scripts/queue folder

===============
Important Notes
===============

------------------
Note about Github
------------------

Do not push original DEMs to github, since most DEMs are quite large, your
commit will be rejected if the file is over the 100 MB (file sizes of
50-100 MB will be given a warning).

------------------
Note about Docker
------------------

The more you run this script, the more memory the docker images will take up
since it appears that each build of an image is kept in like a linked list
cache. It may also take sometime to load up a docker image since it needs to
install dependencies before running a program. But once you have run one of
programs before, it will cache some of the computation when you run it again.
However, this may create another image, which is something to keep in mind if
you're using docker for more than this application. If you want to keep your
other docker stuff, you'll have to manually delete the images. Otherwise, you
can simply run the clean command and it will wipe all docker containers,
images, etc. Because the containers are set to remove themselves after
execution, you just need to get rid of the images.

Remove all docker images:

.. code::

	docker rmi $(docker images -q)

Remove a docker image by tag:

.. code::

	docker rmi test1:ver_number

If you would like to know why we're using docker and some details about how
it works, see the section below labeled "Docker: Isolation"


====================
Running the programs
====================

--------------------
run_programs.sh run
--------------------

When you execute the script with the run command, you will be prompted to provide
how many programs you would like to run, from 0 to 4. If in range and not 0,
you will be prompted with what program would you like to run for each iteration.
If you run more than 1 program, the output for each sequential execution will
feed off each other. For example, if you say you want to run 2 programs,
``extract_tile`` and ``mk_gaz_wrld``, once ``extract_tile`` is finished, the
results will be queued into ``mk_gaz_wrld``. The script allows you to order the
programs however you want, just keep in mind that some permutations aren't
efficient or just redundant. If you want to do all of them in a single run, the
recommended order is 4->3->2->1.

**Note**: ``auto_gen_wrld`` (5) is not streamlined to run as an option in the
``run_programs.sh`` script and does not require docker. The process of running
this script is relatively similar to how to run the ``run_programs.sh``.
See below for more details.

To run the script
------------------

.. code::

	sh run_programs.sh [reset] [queue_reset] [results_reset] [run] [clean]

**Note 1**: Several of these tasks require Docker access and will only work if you are `part of the docker group.`_

**Note 2**: Please run the ``reset``, ``queue_reset``, and ``results_reset``
commands first since there is just a placeholder file in the otherwise empty
folders

Command line argument options
-----------------------------

reset
    Deletes all files from each respective program's nested ``queued_dems`` and ``results`` folders but not ``dem_scripts/queue`` nor ``dem_scripts/results``

queue_reset
    Deletes all files from ``dem_scripts/queue`` folder

results_reset
    Deletes all files from ``dem_scripts/results`` folder 

run
    Runs the script

clean
    Purge your system of docker containers, images, etc.


List of Programs
-----------------

1. get_elev
	Given a DEM (``.tif``), it will output one text file:

        - ``DEM_FILE_NAME_extr_out.txt`` - consists of the decimal coordinates
          (lat, long) of each corner pixel, dimensions of the DEM, and all
          the elevation values

2. mk_gaz_wrld
	Given a DEM (``.tif``), it will output two files:

        - ``DEM_FILE_NAME_resized.tif`` - a downsized dem of a given size
        - ``DEM_FILE_NAME_converted.jpg`` - converted dem to jpg using
          ``DEM_FILE_NAME_resized.tif``

		You can use the ``.jpg`` to create a gazebo world through
		the``auto_gen_wrld``or through manual creation
		(see the "Making a Gazebo World" section).

3. extract_tile
	Given a DEM (``.tif``), it will create DEM tiles (``.tif``) for each part
	of the DEM:

        - ``DEM_FILE_NAME_SOMEX_SOMEY.tif`` - extracted "tile" of a given size

        This functionality is included to keep the right scaling factor when
        trying to create a gazebo world from a DEM. DEMs are quite large and
        represent quite large areas, so to get it into the right format for
        gazebo, we have to downsize or compress it. The available tile sizes
        are: ``513x513``, ``257x257``, or ``129x129``. You could modify this
        code to support other sizes but these are the known sizes that gazebo
        can recognize for a heightmap object.

4. convert2tif
	Given a PDS (``.lbl + ((aux.xml + .jp2) or (.img))``), it outputs:

    	- ``PDS_FILE_NAME.tif`` - converted DEM (``.tif``) of the PDS file

    	Might work with ISIS files (``.cub + .lbl``) but not tested

    	If you encounter errors such as:

    	.. code::

    		band 1: IReadBlock failed at X offset 0, Y offset 0: Failed
        	to read scanline 0

        	or

            	0ERROR 1: Tile part length size inconsistent with stream length

        Either part of the file is corrupted or missing, try using a
        different pds or re-download it

5. auto_gen_wrld
	For each image (``.jpg``) in the ``queue/`` in the ``auto_gen_wrld``,
	it outputs a folder with:

		- ``USER_DEFINED_NAME_world.world`` - a world file
		- ``USER_DEFINED_NAME/`` - a model folder

	These items are piped into to appropriate folders for you. Ideally, use the
	output of ``mk_gaz_wrld`` for input to this program, especially if you want to
	use "Park Ranger" for this (see the "Park Ranger" section for details).

	Things this script does NOT do for you:

		- Check if your image is of the correct size for gazebo
		- Reset the queue
		- Allow for user configuration other than the ``USER_DEFINE_NAME`` or
		  the ``range`` (highest - lowest elevation) factor

	You can of course edit other items after everything is generated but
	be sure to update the world file as well as the sdf file in the respective
	model directory that is made for the heightmap in the ``~/.gazebo/models``.
	You only need to update the tags in common for the heightmap, i.e. don't
	define physics in both the ``.sdf`` and the ``.world`` files.

6. auto_move
    Given a ``SOMENAME_pack/``, which contains a model/ and world file, it moves
    the items to the corresponding folders. It also clears the cache for the
    ``DEM_FILE_IMAGE.jpg`` used to create the terrain but assumes only two items
    in the ``materials/textures/`` of the model.

For more quirky details, see the "Making a Gazebo World" section.

**IMPORTANT NOTE**: Before executing run again, you might want to at least
execute the reset command otherwise it will execute the old jobs you sent to
them as well as the new ones in the queues.

**Output**: Output files can be found in each program's nested results folder
and also the ``dem_scripts/results`` folder (fed all the output files of each
program except ``auto_gen_wrld``). You may have to change the permissions on the
files to edit them.

======================
Making a Gazebo World
======================

After executing the ``mk_gaz_wrld``, you can use the outputted ``.jpg`` or
downsized ``.tif`` or, if you used the ``extract_tile`` program, you can use a
tile ``.tif`` to create a gazebo world. ``auto_gen_wrld`` can do this for you
automatically if you wish to create moon-like of terrain. This section is just
an interpretation of making moon-like terrain and any problems/solutions that
come with it.

**NOTE**: Using a ``.tif`` file might be a bit buggy, mostly the ones that have
elevation values on the extreme ends i.e ``z = 3000`` or ``z = -4927``.
To reduce uncertainty, the code is reflected to expect ``.jpg`` s along with
the extracted elevation data, which is outputted by ``get_elev``. For more
information about this, see the "Park Ranger" section.

Cheatsheet for what each tag means (EZ-RASSOR currently uses version 1.4):

http://sdformat.org/spec

------------
Definitions
------------

Model
	This is essential making the ``.jpg`` into a persistent gazebo world object
	you call in the ``.world`` file. This consists of multiple items, all within
	a ``model_name`` folder:

		- ``model.config`` - References the ``.sdf`` file and has metadata
		- ``model.sdf`` - Reference the path to the DEM ``.jpg`` (creates the
		  geometry of the object) as well as to paths for normal and
		  diffuse texturing
		- ``materials/`` - Stores the jpg and all the other files used for textures
		  or appearance

World
	This is where you define the environment that the rover will load up into

--------------
Gazebo Quirks
--------------

- If you have worlds using different models but use the same ``DEM_FILE.jpg``,
  you have to remove the ``DEM_FILE/`` in ``~/.gazebo/paging/`` when switching
  between worlds.
- If you have worlds sharing the same model but in different configurations
  (i.e. position), you have to remove the ``DEM_FILE/`` in ``~/.gazebo/paging/``
  as well as updating the ``model.sdf`` with these differences when switching
  between worlds. If you don't do this, the world will display the world that
  was ran first out of all the worlds that share the same model.
- World and model/ shouldn't be named the same thing to avoid an error where
  it cannot find the model. There might not be any seen ramifications to the user
  but it shows up when ran with gazebo's ``--verbose``, so it's best
  to be avoided.
- If you encounter black and yellow stripes, you may have to clear the cache for
  the ``.jpg`` used to create the model.

  ``rm -rf ~/.gazebo/paging/DEM_IMAGE_FILE_WITHOUT_EXTENSION``


============
Park Ranger
============

Although you can use either a ``.jpg`` or ``.tif``, we've opted to use
``.jpg`` s so the autonomy code reflects this decision. This is primarily due
to one of our localization estimation methods called park ranger. It depends
upon knowing your elevation and a DEM of the area, so in order to get them in
the same frame, we place the heightmap to make the z at the gazebo origin's to
start at zero elevation. Then, Park Ranger offsets the z values with a text file
equivalent of the data in the ``.tif``, which allows us to simulate
an "altimeter" data without the weirdness of a ``.tif`` heightmap.

NOTE: If you do use the ``.tif`` file with the ``enable_real_odometry`` flag
is set to true, the world state object in the autonomy package will have
an incorrect z value since it derives the elevation as
``gazebo position z + dem middle point elevation``.

To ensure park ranger functionality works, you must do the following things:

	- The ``dem_data/`` in autonomy must have a ``DEM_FILE_NAME_extr_out.txt``
	  for the ``.jpg`` used in the ``.world`` and ``.sdf`` in the model

	- The ``<heightmap><size>`` tag must consist of ``<size> m m range </size>``,
	  where
	  ``range = max_elev - min_elev`` and ``m == m == jpg_dem_width == jpg_dem_length``

If you load up the simulation and the terrain has extreme slopes, that means
it could be one of two things: the ratio of ``range`` to the ``mxm`` of the
heightmap is too large or there is a high density of local max and local mins.
To mitigate either case, lowering the ``range`` value seemed to fix it,
albeit at possibly less accuracy.

==================
Docker: Isolation
==================

-----------
Why Docker
-----------

Because DEM readers aren't built into Ubuntu, we need to use either
an application, driver, or a library that can be used to read them in. In most
applications and libraries, they use a library called GDAL (https://gdal.org/)
as the base for all their functionality. GDAL is "a translator library for
raster and vector geospatial data formats" of which includes support for
PDS (Nasa's Planetary Data System format) and GeoTiff (``.tiff``). Although Gazebo
depends on a GDAL library, those dependancies only let Gazebo read dems and are
not persistent outside of Gazebo. If you search your system for those GDAL
libraries they will show up, but they don't recognize operations such as
gdal_translate. Because of this, when you try to install additional libraries
for development with GDAL, there are dependency conflicts between Gazebo and
GDAL development libraries. So in order to read DEMs without breaking the
environment to run the EZ-RASSOR simulation, we have to isolate it.

We have 3 options: VirtualMachine, Docker, or Anaconda. VMs are quite heavy
since we don't need a whole operating system, just a terminal. Anaconda is
a popular package manager for python and can also provide isolated environments
for python. The main problems with it are that since it doesn't play well with
ROS out of the box and it can install excess packages (mostly data science) we
don't need for our application. Anaconda would be a great choice if you're
already using python for data science stuff but in our case, it's only for
EZ-RASSOR. In hindsight, there is the lighter version of Anaconda called
Miniconda that you could use but learning docker can be applied to more fields
so using docker for only this application isn't as bloatware-y as -conda stuff
is to non-data scientists.


A Note about Python Virtual Environments
-----------------------------------------

Above were the known options when the script was made. If you wish to recreate
the functionality without docker, I recommend python virtual environments.
Below are links that explain it better than me why and when to use python
virtual environment. I attempted to see if you can install GDAL but I ran into
problems with it. I included a link that may fix it but Docker implementation
is good enough for our iteration.

pip vs pyenv vs virtualenv vs anconda:

https://stackoverflow.com/questions/38217545/what-is-the-difference-between-pyenv-virtualenv-anaconda

Python virtual environment:

https://towardsdatascience.com/virtual-environments-104c62d48c54

Installing GDAL in virtual environment (the text is weird on the page):

https://pypi.org/project/pygdal/

Docker vs python virtual environment:

https://coderbook.com/@marcus/should-i-use-virtualenv-or-docker-containers-with-python/

----------------------
Docker Implementation
----------------------

In terms of docker implementation, each program has their own docker image
associated with it (except auto_gen_wrld). If you notice, there is
a Dockerfile.base and a Dockerfile.child file rather than the standard one
Dockerfile per directory. The Dockerfile.base and entrypoint.sh are just for
setting up the program to run as a local user vs root. Even though that a docker
container is used for isolation, it's insecure to run as root.

Why to not run as root (there are other articles that mention this):

https://americanexpress.io/do-not-run-dockerized-applications-as-root/

Another thing that we do that is not standard, we use bind mounts to mount
a host directory inside a docker container. The most popular way to store data
with Docker is through volumes but isn't the ideal for our application. Volumes
are usually the recommended choice since they are more secure and a great way
to transfer data from container to container. Although we could store our data
in volumes and pass it to other containers created if a user runs more than one
of the programs in the script, we ultimately need to get the results back to
the host, which isn't possible or at least quite difficult to do. This makes
bind mounts the best choice in this case.

Full comparison of storage formats in Docker:

https://docs.docker.com/storage/

.. _`part of the docker group.`: https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user

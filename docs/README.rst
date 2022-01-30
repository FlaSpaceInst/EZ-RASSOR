EZ-RASSOR
---------
|build badge| |style badge|

The EZ-RASSOR (EZ Regolith Advanced Surface Systems Operations Robot) is an inexpensive, autonomous, regolith-mining robot designed to mimic the look and abilities of NASA’s RASSOR on a smaller scale. The primary goal of the EZ-RASSOR is to provide a demonstration robot for visitors at the Kennedy Space Center. The EZ-RASSOR can:

- Rove across light-to-moderate terrain
- Collect regolith in rotating drums
- Return regolith to hoppers located away from dig sites
- Execute pre-planned routines
- Autonomously navigate around possible obstructions
- Cooperate in a swarm of other EZ-RASSORs

For more information, our `wiki`_ contains a high-level overview of the EZ-RASSOR and its many components.

**POTENTIAL CONTRIBUTORS:** check out the `contributing guidelines`_ and the `license`_.

INSTALLATION PREREQUISITES
--------------------------
- `ROS Melodic`_
- `Python 2.7`_
- `Pip`_
- `rosdep`_
- `build-essential`_

TYPICAL INSTALLATION
--------------------
First, clone this repository with ``git``.

.. code-block:: bash

    git clone https://github.com/FlaSpaceInst/EZ-RASSOR.git
    cd EZ-RASSOR 

Next if you plan to use the paver arm rover model please clone the following git

.. code-block:: bash

    git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
    
And place the cloned folder inside EZ-RASSOR/Packages/Simulation directory

Then, let the ``develop.sh`` script do the heavy lifting! On Ubuntu Xenial or Ubuntu Bionic, creating a catkin workspace and building all packages is achieved with these commands:

.. code-block:: bash

  # By default, all ROS packages in the *packages* folder will be installed
  sh develop.sh setup
  sh develop.sh link
  sh develop.sh resolve
  sh develop.sh build
  sh develop.sh install
  ** RESTART TERMINAL **

If you encounter ``Sub-process /usr/bin/dpkg returned an error code...``, try to fix the broken install with the following command, then rerun the original command:

.. code-block:: bash

  sudo apt --fix-broken install
  ** RERUN ORIGINAL COMMAND **
  
Everything's installed now! Proceed to the `usage`_ section.

CUSTOMIZED INSTALLATION
-----------------------
If you want to install specific EZ-RASSOR packages, you can use the same develop.sh script:

Make sure you have already run the setup command at least once:

.. code-block:: bash

  sh develop.sh setup

Then, you can call the relink function and use ``-o`` to pass in the package name(s) you would like to install:

.. code-block:: bash

  sh develop.sh relink -o ezrassor_sim_control ezrassor_sim_description ezrassor_sim_gazebo
  sh develop.sh build
  sh develop.sh install
  
Alternatively, you can also call the relink function and use the ``-e`` flag to make the script install all *but* the specified package(s):

.. code-block:: bash

  sh develop.sh relink -e ezrassor_swarm_control

INSTALLATION FOR OBJECT DETECTION
---------------------------------
In order for the object detection script to work, you may need to ensure you have certain python libraries installed properly. First you will need to upgrade pip before installing each library

.. code-block:: bash

  pip install --upgrade pip

Next, make sure you uninstall tensorflow, numpy and/or opencv if you have already installed them, and reinstall them.

.. code-block:: bash

  pip uninstall <library>
  pip install <library>
  #Example
  pip uninstall tensorflow
  pip install tensorflow

You will need to upgrade tensorflow to the current version to avoid errors when running the script.

.. code-block:: bash

  pip install --upgrade tensorflow
  pip install --upgrade --force-reinstall tensorflow

Finally, opencv requires that you have the frozen weights and the configuration file for the trained model in order to load and run the input image through it. You must install these two files for this to work.

- `frozen_inference_graph.pb`_
- `graph.pbtxt`_

Then make sure you move these files to the ``FSI-RE-RASSOR-Arm\packages\autonomy\ezrassor_arm_autonomous_control\source\ezrassor_arm_autonomous_control`` directory.

USAGE
-----
The EZ-RASSOR is controlled via a collection of *launch files*. These files contain lists of commands that start up the robot's systems and the simulation environment. They are read, understood, and executed by a core ROS utility called ``roslaunch``, whose general syntax is as follows:

.. code-block:: bash

  roslaunch <package> <launch file> [arguments...]
  
Each launch file is located in one of our packages, and the most important launch files are located in the ``ezrassor_launcher`` package. To learn more about a specific launch file, visit that launch file's package's `wiki`_ page (via the navigation menu on the right). Here are some example commands that show launch files in action:

.. code-block:: bash

  # Launch the simulation with a single robot controlled by the mobile app.
  roslaunch ezrassor_launcher configurable_simulation.launch control_methods:=app
  
  # Launch the simulation with a single robot controlled by an autonomous loop.
  roslaunch ezrassor_launcher configurable_simulation.launch control_methods:=autonomy
  
  # Launch the simulation with a single robot using the paver arm and controlled by an autonomous loop.
  roslaunch ezrassor_launcher configurable_simulation.launch control_methods:=autonomy rover_model:=paver_arm
  
  # Launch the simulation with two robots, both controlled by gamepads, on the moon.
  roslaunch ezrassor_launcher configurable_simulation.launch \
      control_methods:=gamepad \
      world:=moon \
      robot_count:=2 \
      joysticks:="0 1" \
      spawn_x_coords:="-1 1" \
      spawn_y_coords:="1 -1"
      
  # Launch the communication system in dual mode: manual and autonomous control together.
  roslaunch ezrassor_launcher configurable_communication.launch control_methods:="app gamepad autonomy"
  
Please read the `wiki page for the ezrassor_launcher`_ to learn more about what the main launch files can do.

AUTHORS
-------
**EZ-RASSOR 1.0 Team**

- `Sean Rapp`_
- `Ron Marrero`_
- `Tiger Sachse`_
- `Tyler Duncan`_
- `Samuel Lewis`_
- `Harrison Black`_
- `Camilo Lozano`_
- `Chris Taliaferro`_
- `Cameron Taylor`_
- `Lucas Gonzalez`_

**EZ-RASSOR 2.0 (GPS-Denied Autonomous Navigation) Team**

- `Jordan Albury`_
- `Shelby Basco`_
- `John Hacker`_
- `Michael Jimenez`_
- `Scott Scalera`_

**EZ-RASSOR 2.0 (Swarm Control & Management) Team**

- `Daniel Silva`_
- `Chin Winn`_
- `Martin Power`_
- `Daniel Simoes`_
- `Autumn Esponda`_

**EZ-RASSOR 3.0 (Swarm Build Pad) Team**

- `Hung Nguyen`_
- `Coy Torreblanca`_
- `Stanley Minervini`_
- `Richard Malcolm`_
- `Camry Artalona`_

.. |build badge| image:: https://github.com/FlaSpaceInst/EZ-RASSOR/workflows/Build/badge.svg
    :target: https://github.com/FlaSpaceInst/EZ-RASSOR/actions
.. |style badge| image:: https://img.shields.io/badge/Code%20Style-black-000000.svg
    :target: https://github.com/psf/black
.. _`wiki`: https://github.com/FlaSpaceInst/EZ-RASSOR/wiki
.. _`contributing guidelines`: CONTRIBUTING.rst
.. _`license`: LICENSE.txt
.. _`usage`: README.rst#Usage
.. _`wiki page for the ezrassor_launcher`: https://github.com/FlaSpaceInst/EZ-RASSOR/wiki/ezrassor_launcher
.. _`Sean Rapp`: https://github.com/shintoo
.. _`Ron Marrero` : https://github.com/CSharpRon
.. _`Tiger Sachse` : https://github.com/tgsachse
.. _`Tyler Duncan` : https://github.com/Tduncan13
.. _`Samuel Lewis` : https://github.com/BrainfreezeFL
.. _`Harrison Black` : https://github.com/HarrisonWBlack
.. _`Camilo Lozano` : https://github.com/camilozano
.. _`Chris Taliaferro` : https://github.com/Hansuto
.. _`Cameron Taylor` : https://github.com/CameronTaylorFL
.. _`Lucas Gonzalez` : https://github.com/gonzalezL
.. _`Jordan Albury` : https://github.com/jalbury
.. _`Shelby Basco` : https://github.com/blicogam
.. _`John Hacker` : https://github.com/JHacker997
.. _`Michael Jimenez` : https://github.com/Mjimenez01
.. _`Scott Scalera` : https://github.com/ScottCarL
.. _`Daniel Silva` : https://github.com/danielzgsilva
.. _`Chin Winn` : https://github.com/wchinny
.. _`Martin Power` : https://github.com/martinpower
.. _`Daniel Simoes` : https://github.com/RuptorT
.. _`Autumn Esponda` : https://github.com/autumnesponda
.. _`Hung Nguyen` : https://github.com/hungjn
.. _`Coy Torreblanca` : https://github.com/Coy-Torreblanca
.. _`Stanley Minervini` : https://github.com/StanleyMine
.. _`Richard Malcolm` : https://github.com/RichardMalcolm-CS
.. _`Camry Artalona` : https://github.com/cmryArt
.. _`ROS Melodic` : http://wiki.ros.org/melodic/Installation/Ubuntu
.. _`Python 2.7` : https://www.python.org/download/releases/2.7/
.. _`Pip` : https://pip.pypa.io/en/stable/installing/
.. _`rosdep` : http://wiki.ros.org/rosdep
.. _`build-essential` : https://packages.ubuntu.com/bionic/build-essential
.. _`frozen_inference_graph.pb` : http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_v2_coco_2018_01_28.tar.gz
.. _`graph.pbtxt` : https://github.com/opencv/opencv_extra/blob/master/testdata/dnn/faster_rcnn_inception_v2_coco_2018_01_28.pbtxt

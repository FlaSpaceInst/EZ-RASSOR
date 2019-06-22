EZ-RASSOR
---------
The EZ-RASSOR (EZ Regolith Advanced Surface Systems Operations Robot) is an inexpensive, autonomous, regolith-mining robot designed to mimic the look and abilities of NASA’s RASSOR on a smaller scale. The primary goal of the EZ-RASSOR is to provide a demonstration robot for visitors at the Kennedy Space Center. The EZ-RASSOR can:

- Rove across light-to-moderate terrain
- Collect regolith in rotating drums
- Return regolith to hoppers located away from dig sites
- Execute pre-planned routines
- Autonomously navigate around possible obstructions
- Cooperate in a swarm of other EZ-RASSORs

For more information, our `wiki`_ contains a high-level overview of the EZ-RASSOR and its many components, and our `blue paper`_ goes even further in depth!

If you'd like to contribute, check out the `contributing guidelines`_ and the `license`_.

TYPICAL INSTALLATION
--------------------
Clone this repository with ``git`` and include the ``--recursive`` flag. This tells ``git`` to also clone the submodules that this project relies on.
::
  git clone https://github.com/FlaSpaceInst/EZ-RASSOR.git --recursive
  cd EZ-RASSOR

Next, let the ``install.sh`` script do the heavy lifting! A typical installation on Ubuntu Xenial or Ubuntu Bionic is achieved with these commands:
::
  sh install.sh ros
  ** RESTART TERMINAL **
  sh install.sh tools
  sh install.sh packages
  ** RESTART TERMINAL **
  
You're all set! Proceed to the `usage`_ section.

CUSTOMIZED INSTALLATION
-----------------------
Before performing a custom installation, you will need to install `ROS manually`_. You'll also want to install the `ROS build tools`_.

The installation script's general syntax looks like this:
::
  sh install.sh <software> [arguments...]
  
All of the following are valid ``<software>`` options:

``help``
  Display a help menu.
``ros``
  Install the ROS suite of software. This software is required to operate the EZ-RASSOR (and must be installed first). This script will automatically install ROS for Ubuntu Xenial and Ubuntu Bionic. For all other systems, ROS must be installed manually. After installing ROS with this script, **you must restart your terminal before proceeding**.
``tools``
  Install some ROS build tools that are required to build the EZ-RASSOR core packages. Again, this suite of software can only be installed automatically on Ubuntu Xenial and Ubuntu Bionic. For all other systems, you must install these tools manually.
``packages [-e, --except <packages...> | -o, --only <packages...>]``
  Install the core EZ-RASSOR packages. After installing these packages, **you must restart your terminal for changes to take effect**. Ignore specific packages with the ``-e`` or ``--except`` flag. Install specific packages with the ``-o`` or ``--only`` flag.
  
Once you have ROS and the ROS build tools installed, you can install any combination of our packages. Here are some examples:
::
  # Install all packages except 'ezrassor_autonomous_control'
  sh install.sh packages --except ezrassor_autonomous_control

  # Install all packages except the simulation packages.
  sh install.sh packages --except ezrassor_sim_control ezrassor_sim_description ezrassor_sim_gazebo
  
  # Install only the communication packages.
  sh install.sh packages --only ezrassor_controller_server ezrassor_joy_translator ezrassor_topic_switch
  
USAGE
-----


AUTHORS
-------
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

.. _`wiki`: https://github.com/FlaSpaceInst/EZ-RASSOR/wiki
.. _`blue paper`: BLUE_PAPER.pdf
.. _`contributing guidelines`: CONTRIBUTING.rst
.. _`license`: LICENSE.txt
.. _`usage`: README.rst#Usage
.. _`ROS manually`: http://wiki.ros.org/ROS/Installation
.. _`ROS build tools`: http://wiki.ros.org/kinetic/Installation/Source#Prerequisites
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

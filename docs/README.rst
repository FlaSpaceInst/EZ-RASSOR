EZ-RASSOR
---------
The EZ-RASSOR (EZ Regolith Advanced Surface Systems Operations Robot) is an inexpensive, autonomous, regolith-mining robot designed to mimic the look and abilities of NASA’s RASSOR on a smaller scale. The primary goal of the EZ-RASSOR is to provide a demonstration robot for visitors at the Kennedy Space Center. The EZ-RASSOR can:

- Rove across light-to-moderate terrain
- Collect regolith in rotating drums
- Return regolith to hoppers located away from dig sites
- Execute pre-planned routines
- Autonomously navigate around possible obstructions
- Cooperate in a swarm of other EZ-RASSORs

This repository contains all of the code for the EZ-RASSOR as well as supporting documentation.

INSTALLATION
------------
Clone this repository with ``git`` to begin the installation process. You must use the ``--recursive`` flag when you clone with ``git``. This tells ``git`` to also clone submodules that this project relies on.
::
  git clone https://github.com/FlaSpaceInst/NASA-E-RASSOR-Team.git --recursive
  cd NASA-E-RASSOR-Team

Next, let the ``install.sh`` script do the heavy lifting. This script's general syntax looks like this:
::
  sh install.sh <software> [arguments...]
  
It is able to install all of the following:

``ros``
  Install the ROS suite of software. This software is required to operate the EZ-RASSOR (and must be installed first). This script will automatically install ROS for Ubuntu Xenial and Ubuntu Bionic. For all other systems, check out these `wiki pages`_ to install ROS manually. After installing ROS with this script, **you must restart your terminal before proceeding**.
``tools``
  Install some ROS build tools that are required to build the EZ-RASSOR core packages. Again, this suite of software can only be installed automatically on Ubuntu Xenial and Ubuntu Bionic. For all other systems, check out this `wiki section`_ to install the build tools manually.
``packages [-e, --except <packages...> | -o, --only <packages...>]``
  Install core EZ-RASSOR packages. After installing these packages, **you must restart your terminal for changes to take effect**. Ignore specific packages with the ``-e`` or ``--except`` flag. Install specific packages with the ``-o`` or ``--only`` flag.

Here are some example installations:
::
  # Installing on Ubuntu Xenial with all EZ-RASSOR packages.
  git clone https://github.com/FlaSpaceInst/NASA-E-RASSOR-Team.git --recursive
  cd NASA-E-RASSOR-Team
  sh install.sh ros
  ** RESTART TERMINAL **
  sh install.sh tools
  sh install.sh packages
  ** RESTART TERMINAL **
  
  # Installing on Ubuntu Bionic without the ezrassor_autonomous_control package.
  git clone https://github.com/FlaSpaceInst/NASA-E-RASSOR-Team.git --recursive
  cd NASA-E-RASSOR-Team
  sh install.sh ros
  ** RESTART TERMINAL **
  sh install.sh tools
  sh install.sh packages --except ezrassor_autonomous_control
  ** RESTART TERMINAL **
  
  # Installing on Raspbian Jessie without any simulation packages.
  git clone https://github.com/FlaSpaceInst/NASA-E-RASSOR-Team.git --recursive
  cd NASA-E-RASSOR-Team
  ** INSTALL ROS AND BUILD TOOLS SEPARATELY **
  sh install.sh packages --except ezrassor_sim_control ezrassor_sim_description ezrassor_sim_gazebo
  ** RESTART TERMINAL **
  
USAGE
-----

WIKI
----
Our `wiki`_ contains lots of information about our project! Check it out, or read our `blue paper`_.

CONTRIBUTIONS
-------------
Take a look at the `contributing guidelines`_ if you'd like to help develop this project!

Also be sure to review the `license`_.

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

.. _`wiki pages`: http://wiki.ros.org/ROS/Installation
.. _`wiki section`: http://wiki.ros.org/kinetic/Installation/Source#Prerequisites
.. _`wiki`: https://github.com/FlaSpaceInst/NASA-E-RASSOR-Team/wiki
.. _`blue paper`: BLUE_PAPER.pdf
.. _`contributing guidelines`: CONTRIBUTING.rst
.. _`license`: LICENSE.txt
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

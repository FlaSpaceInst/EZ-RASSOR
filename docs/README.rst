EZ-RASSOR
----
The EZ-RASSOR (Easy Regolith Advanced Surface Systems Operations Robot) is an inexpensive, autonomous, regolith-mining robot designed to mimic the look and functionality of NASA’s RASSOR on a smaller scale. The primary goal of the EZ-RASSOR is to provide a functional demonstration of the RASSOR for visitors at the Kennedy Space Center. The EZ-RASSOR can:

- Rove across light-to-moderate terrain
- Collect regolith in rotating drums
- Execute pre-planned routines
- Return regolith to hoppers located away from dig sites
- Autonomously navigate around possible obstructions
- Cooperate in a swarm of other EZ-RASSORs

This repository contains all of the code for the EZ-RASSOR, its simulations, control apps, and its RC demonstration car called the EZRC.

DEVELOPMENT
----
This repository contains a script, `ezrassor.sh`, that makes improving this software easier and more straightforward for developers. It is run using this command:
::
  bash ezrassor.sh [--flag] [args]
  
The flags supported by this script are listed below:
 
``-i, --install [collections]``
  Install one or many collections of software that are needed by portions of this project. The current collections available are ``ezrc``, ``ros``, ``devtools``, ``ai``, and ``swarm``.
``-c, --catkin``
  Set up a Catkin workspace in your home directory to develop and compile ROS nodes. By default, this workspace is named ``.workspace``.
``-n, --new [superpackage] [package] [dependencies]``
  Create a new ROS ``package`` in the ``packages`` folder, under the appropriate ``superpackage``. If the superpackage doesn't exist it is created. All arguments after package are passed to ``catkin_create_pkg`` (these arguments are usually dependencies of the package). The newly created package is then symlinked into your workspace's ``src`` folder. If you've never run the ``--catkin`` command ensure to do that before trying to make a new package, otherwise you won't have a workspace to develop in.
``-l, --link``
  Create a symlink between all packages in all superpackages in the ``packages`` directory and the ``src`` directory of your workspace. This is necessary after creating a new workspace, or if you've renamed/reorganized the packages in ``packages``. If you've done this, you'll probably want to ``--purge`` first (see below).
``-p, --purge``
  Remove all symlinked packages from ``src``.
``-b, --build``
  Call `catkin_make` in your workspace.
``-s, --start [graph]``
  Fire up a ROS graph. Available ROS graphs are ``ezrc``, ``control``, ``gazebo``, ``rviz``, ``slam-core``, and ``slam-viewer``.
``-k, --kill``
  Kill all running ROS nodes and `roscore`.

AUTHORS
----
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

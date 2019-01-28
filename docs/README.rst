EZ-RASSOR
----
The EZ-RASSOR (EZ Regolith Advanced Surface Systems Operations Robot) is an inexpensive, autonomous, regolith-mining robot designed to mimic the look and abilities of NASA’s RASSOR on a smaller scale. The primary goal of the EZ-RASSOR is to provide a functioning demonstration robot for visitors at the Kennedy Space Center. The EZ-RASSOR can:

- Rove across light-to-moderate terrain
- Collect regolith in rotating drums
- Return regolith to hoppers located away from dig sites
- Execute pre-planned routines
- Autonomously navigate around possible obstructions
- Cooperate in a swarm of other EZ-RASSORs

This repository contains all of the code for the EZ-RASSOR, its simulations, control applications, and its RC demonstration car called the EZRC.

DEVELOPMENT
----
This repository contains a script, ``ezrassor.sh``, that helps developers improve this software with ease. It's general syntax looks like this:
::
  bash ezrassor.sh <--flag> [args]
  
The flags supported by the script are listed below:
 
``-i, --install <collections...>``
  Install one or many collections of software associated with this project. The current collections available are ``ezrc``, ``ros``, ``devtools``, ``ai``, and ``swarm``.
``-c, --catkin``
  Set up a Catkin workspace in your home directory to develop and compile ROS nodes. This workspace is named ``.workspace`` by default.
``-n, --new <superpackage> <package> [dependencies...]``
  Create a new ROS package in the ``packages`` folder, under the appropriate superpackage. If the superpackage doesn't exist it is created. All arguments after package are passed to ``catkin_create_pkg`` (these arguments are usually dependencies of the package). The newly created package is then symlinked into your workspace's ``src`` folder. If you've never run the ``--catkin`` command ensure that you do that before trying to make a new package, otherwise you won't have a workspace to develop in!
``-l, --link [-i, --ignore <packages...>]``
  Create a symlink from all packages in the ``packages`` directory to the ``src`` directory of your workspace. This is necessary after creating a new workspace, or if you've renamed/reorganized the packages in ``packages``. If you've done this, you'll want to ``--purge`` before running this command (see below), otherwise your ``src`` directory could contain broken symlinks to removed/renamed packages. Ignore specific packages with the ``-i`` or ``--ignore`` flag.
``-p, --purge``
  Remove all symlinked packages from ``src``.
``-r, --relink [-i, --ignore <packages...>]``
  Purge all symlinked packages from ``src``, and then link all packages in ``packages``. Ignore specific packages with the ``-i`` or ``--ignore`` flag.
``-b, --build``
  Call ``catkin_make`` in your workspace.
``-s, --start <graph>``
  Fire up a ROS graph. Available ROS graphs are ``ezrc``, ``control``, ``gazebo``, ``rviz``, ``slam-core``, and ``slam-viewer``.
``-k, --kill``
  Kill all running ROS nodes and `roscore`.

EXAMPLES
----
Here are some example commands to get started.
::
  # Install the ROS and EZRC software collections, then create a Catkin
  # workspace and link all existing packages in the repository.
  bash ezrassor.sh --install ros ezrc
  bash ezrassor.sh --catkin
  bash ezrassor.sh --link
  
  # Create a new package in the superpackage 'ezrc' called 'ezrc_cameras'.
  bash ezrassor.sh --new ezrc ezrc_cameras
  
  # Fire up the EZRC ROS graph.
  bash ezrassor.sh --start ezrc
  
  # Kill all running ROS nodes.
  bash ezrassor.sh --kill
  
  # Build the contents of 'src' in your Catkin workspace.
  bash ezrassor.sh --build

  # Relink all packages except the 'depreciated' package.
  bash ezrassor.sh --relink --ignore depreciated

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

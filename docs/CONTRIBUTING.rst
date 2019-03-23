CONTRIBUTING NOTICE
----
This project is free and open-source under the `LICENSE license`_. Anyone can fork this repository and submit a pull request, however all pull requests are subject to review and approval by this project's `authors`_. All merged code becomes part of this project, and thus is subject to the same license as the rest of the code in this project. Any code in this repository may be deleted, modified, or rewritten at any time. **Ultimately the authors of this project, the Florida Space Institute, and NASA have final control over this project's code.** By submitting a pull request, you voluntarily surrender all the rights you possess over your code to the Florida Space Institute, NASA, and the authors of this project (with the good-faith expectation that your contributions will be adaquately credited to you). New authors may be named periodically, depending on contribution size and project demands.

DEVELOPMENT INSTRUCTIONS
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
``-l, --link [-i, --ignore <packages...> | -o, --only <packages...>]``
  Create a symlink from all packages in the ``packages`` directory to the ``src`` directory of your workspace. This is necessary after creating a new workspace, or if you've renamed/reorganized the packages in ``packages``. If you've done this, you'll want to ``--purge`` before running this command (see below), otherwise your ``src`` directory could contain broken symlinks to removed/renamed packages. Ignore specific packages with the ``-i`` or ``--ignore`` flag. Relink specific packages with the ``-o`` or ``--only`` flag. When linking specific packages, all other packages are purged.
``-p, --purge``
  Remove all symlinked packages from ``src``.
``-r, --relink [-i, --ignore <packages...> | -o, --only <packages...>]``
  Purge all symlinked packages from ``src``, and then link all packages in ``packages``. Ignore specific packages with the ``-i`` or ``--ignore`` flag. Relink specific packages with the ``-o`` or ``--only`` flag. When relinking specific packages, all other packages are purged.
``-b, --build``
  Call ``catkin_make`` in your workspace.
``-s, --start <graph>``
  Fire up a ROS graph. Available ROS graphs are ``ezrc``, ``control``, ``gazebo``, ``rviz``, ``slam-core``, and ``slam-viewer``. You must start ``control`` before ``slam-core`` and ``slam-viewer``.
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

  # Link all packages except the 'depreciated' package.
  bash ezrassor.sh --link --ignore depreciated

  # Relink only the 'ezrc_moving_parts' and 'lsd_slam' packages.
  bash ezrassor.sh --relink -o ezrc_moving_parts lsd_slam
  
.. _`LICENSE license`: LICENSE.txt
.. _`authors`: https://github.com/FlaSpaceInst/NASA-E-RASSOR-Team/blob/master/docs/README.rst#authors

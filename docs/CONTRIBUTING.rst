CONTRIBUTING NOTICE
----
This project is free and open-source under the `MIT license`_. Anyone can fork this repository and submit a pull request, however all pull requests are subject to review and approval by this project's `authors`_. All merged code becomes part of this project, and thus is subject to the same license as the rest of the code in this project. Any code in this repository may be deleted, modified, or rewritten at any time. **Ultimately the authors of this project, the Florida Space Institute, and NASA have final control over this project's code.** By submitting a pull request, you voluntarily surrender all the rights you possess over your code to the Florida Space Institute, NASA, and the authors of this project (with the good-faith expectation that your contributions will be adequately credited to you). New authors may be named periodically, depending on contribution size and project demands.

DEVELOPMENT INSTRUCTIONS
----
Before you begin developing, you must install ROS and the ROS build tools. If you are developing on Ubuntu Xenial or Ubuntu Bionic, you can install these software packages easily with the ``install.sh`` script. You also probably want to install this project's submodules (currently only Viso2) permanently. Execute these commands to get started:
::
  sh install.sh ros
  sh install.sh tools
  ** RESTART TERMINAL **
  sh install.sh packages --only viso2_ros libviso2
  ** RESTART TERMINAL **
  
If you need more installation information or are using a different operating system, please see the `README`_.

The next tool you'll want to get familiar with is the ``develop.sh`` script, which helps developers improve this software with ease. It's general syntax looks like this:
::
  sh develop.sh <mode> [arguments...]
  
The modes supported by the script are listed below:
 
``setup``
  Set up a Catkin workspace in your home directory to develop and compile ROS nodes. This workspace is named ``.workspace`` by default. This mode also sources the new workspace's ``build`` directory in your shell's configuration files.
``new <superpackage> <package> [dependencies...]``
  Create a new ROS package in the ``packages`` folder, under the appropriate superpackage. If the superpackage doesn't exist it is created. All arguments after ``package`` are passed to ``catkin_create_pkg`` (these arguments are usually dependencies of the package). The newly created package is then symlinked into your workspace's ``src`` folder. If you've never run ``setup`` ensure that you do that before trying to make a new package, otherwise you won't have a workspace to develop in!
``link [-e, --except <packages...> | -o, --only <packages...>]``
  Create a symlink from all packages in the ``packages`` directory to the ``src`` directory of your workspace (so that you can build and test your software, without having to copy it into the workspace each time). You should execute this mode after creating a new workspace, or if you've renamed/reorganized the packages in ``packages``. If you've done this, you'll want to ``purge`` before running this command (see below), otherwise your ``src`` directory could contain broken symlinks to removed/renamed packages. Exclude specific packages with the ``-e`` or ``--except`` flag. Link specific packages with the ``-o`` or ``--only`` flag.
``purge``
  Remove all symlinked packages from ``src``.
``relink [-e, --except <packages...> | -o, --only <packages...>]``
  Purge all symlinked packages from ``src``, and then link all packages in ``packages``. Ignore specific packages with the ``-e`` or ``--except`` flag. Relink specific packages with the ``-o`` or ``--only`` flag.
``resolve``
  Install all required dependencies for currently linked packages.
``build``
  Call ``catkin_make`` in your workspace.
``install``
  Install all built packages into the install target in your workspace (via ``catkin_make install``).
``kill``
  Kill all running ROS nodes and ``roscore``.
``test``
  Run integration tests for all linked packages.
``help``
  Display a help menu.

Here are some example commands to get started.
::
  # Set up a new Catkin workspace.
  sh develop.sh setup
  
  # Create a new package in the superpackage 'autonomy' called 'ezrassor_swarm'.
  sh develop.sh new autonomy ezrassor_swarm
  
  # Link only your new package and 'ezrassor_launcher', plus install dependencies.
  sh develop.sh link --only ezrassor_swarm ezrassor_launcher
  sh develop.sh resolve

  # Build your linked packages.
  sh develop.sh build

  # Something went wrong... relink all packages except 'ezrassor_swarm'.
  sh develop.sh relink --except ezrassor_swarm

  # Build and install your linked packages.
  sh develop.sh build
  sh develop.sh install

.. _`MIT license`: LICENSE.txt
.. _`authors`: https://github.com/FlaSpaceInst/NASA-E-RASSOR-Team/blob/master/docs/README.rst#authors
.. _`README`: README.rst#INSTALLATION

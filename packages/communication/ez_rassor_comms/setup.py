"""Setup the ez_rassor_comms ROS package.

Written by Tiger Sachse.
"""
import distutils.core
import catkin_pkg.python_setup

setup_arguments = catkin_pkg.python_setup.generate_distutils_setup(
    packages=(
        "ez_rassor_comms",
    ),
    package_dir={
       "" : "source",
    },
)
distutils.core.setup(**setup_arguments)

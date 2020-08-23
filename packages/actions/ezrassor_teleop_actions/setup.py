"""Setup the ezrassor_teleop_actions ROS package.

Written by Ron Marrero.
"""
import distutils.core
import catkin_pkg.python_setup

setup_arguments = catkin_pkg.python_setup.generate_distutils_setup(
    packages=("ezrassor_teleop_actions",),
    package_dir={"": "source"},
)

distutils.core.setup(**setup_arguments)

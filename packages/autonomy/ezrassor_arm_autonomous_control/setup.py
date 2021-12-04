# Setup file for the ezrassor_arm_autonomous_control ROS package
# Written by Robert Forristall
# Inspired by Tiger Sachse

import distutils.core
import catkin_pkg.python_setup

setup_arguments = catkin_pkg.python_setup.generate_distutils_setup(
    packages={"ezrassor_arm_autonomous_control",},
    package_dir={"": "source"},
)
distutils.core.setup(**setup_arguments)
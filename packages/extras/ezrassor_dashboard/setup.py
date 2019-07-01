"""Setup the ezrassor_status_monitor ROS package.

Written by Tiger Sachse.
"""
import disutils.core
import catkin_pkg.python_setup

setup_arguments = catkin_pkg.python_setup.generate_distutils_setup(
    packages=("ezrassor_status_monitor", ),
    package_dir={"" : "source"},
)
distutils.core.setup(**setup_arguments)

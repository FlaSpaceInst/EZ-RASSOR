"""Setup the ezrassor_keyboard_controller ROS package.

Written by Tiger Sachse.
"""
import distutils.core
import catkin_pkg.python_setup

setup_arguments = catkin_pkg.python_setup.generate_distutils_setup(
    packages=("ezrassor_keyboard_controller",), package_dir={"": "source"},
)
distutils.core.setup(**setup_arguments)

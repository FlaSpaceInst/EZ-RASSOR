"""Setup the EZRC ROS package.

Written by Tiger Sachse.
"""
import distutils.core
import catkin_pkg.python_setup

setup_arguments = catkin_pkg.python_setup.generate_distutils_setup(
    packages=(
        "ezrc",
    ),
    package_dir={
       "" : "source",
    },
    requires=(
        "RPi.GPIO",
        "Adafruit_PCA9685",
    ),
)
distutils.core.setup(**setup_arguments)

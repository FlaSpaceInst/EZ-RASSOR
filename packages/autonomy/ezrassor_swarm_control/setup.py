"""Setup the ezrassor_swarm_control ROS package.
"""

import distutils.core
import catkin_pkg.python_setup

setup_arguments = catkin_pkg.python_setup.generate_distutils_setup(
    packages=("ezrassor_swarm_control", ),
    package_dir={"" : "source"},
)
distutils.core.setup(**setup_arguments)

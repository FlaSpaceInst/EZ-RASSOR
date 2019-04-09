import distutils.core
import catkin_pkg.python_setup

setup_arguments = catkin_pkg.python_setup.generate_distutils_setup(
    packages=(
        "ezrassor_sim_control",
    ),
    package_dir={
        "" : "source",
    },
)
distutils.core.setup(**setup_arguments)

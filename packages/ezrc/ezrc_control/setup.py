from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['ezrc_control'],
    package_dir={'': 'packages'})

setup(**setup_args)

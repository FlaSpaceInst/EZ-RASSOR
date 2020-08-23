#!/bin/sh -l

cd /ez-rassor

# Install the EZRASSOR from source.
sh develop.sh setup
sh develop.sh link
sh develop.sh build
sh develop.sh install

# Source the installation.
. /root/ezrassor_ws/install/setup.sh

# Run the test function (uses catkin_make run_tests).
sh develop.sh test

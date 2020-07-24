#!/bin/sh -l

cd /ez-rassor

# Go through the setup commands
sh develop.sh setup
sh develop.sh link
sh develop.sh build

# Run the test function (uses catkin_make run_tests)
sh develop.sh test
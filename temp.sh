#!/bin/bash
cat docs/CONTRIBUTING.rst \
    | grep '^``' -A 1 \
    | sed 's/  //g' \
    | sed 's/\n/\n    /g'
    #| fold -s -w 65
#    | sed 's/``//g'

#!/usr/bin/env bash
# Compile+run the ROS-free core unit tests.
set -e
cd ~/gdn_workspace/src/gdn_fusion
g++ -std=c++17 -O2 -Wall -Wextra -Wno-maybe-uninitialized \
    -I include -I /usr/include/eigen3 \
    src/selftest.cpp -o /tmp/gdn_selftest
/tmp/gdn_selftest

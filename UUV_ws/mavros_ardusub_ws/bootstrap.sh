#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-mavlink \
  ros-jazzy-geographic-msgs \
  ros-jazzy-ament-cmake-google-benchmark \
  libasio-dev \
  libgeographiclib-dev \
  geographiclib-tools

sudo ros2 run mavros install_geographiclib_datasets.sh
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

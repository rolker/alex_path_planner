#!/bin/bash
# Run this script to generate HTML and LaTeX docs for the path planner and controller. This script will attempt to
# install doxygen if it is not already installed.
REQUIRED_PKG="doxygen"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi
cd alex_path_planner_common
doxygen
cd ../alex_path_planner
doxygen
cd ../../alex_mpc
doxygen
cd ../alex_path_planner
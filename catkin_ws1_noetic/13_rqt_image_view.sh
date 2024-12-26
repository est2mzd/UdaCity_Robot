#!/bin/bash
set -e

cd ~/catkin_ws/

# Interacting with the arm
rqt_image_view /rgb_camera/image_view

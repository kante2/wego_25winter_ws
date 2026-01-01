#!/bin/bash
# Quick Deploy Script for Traffic Light Integration
# Run this on the robot after git pull

set -e

echo "=================================================="
echo "WEGO Traffic Light Integration - Deploy Script"
echo "=================================================="

cd ~/catkin_ws

echo ""
echo "Step 1: Cleaning old build artifacts..."
catkin_make clean

echo ""
echo "Step 2: Building ROS packages..."
catkin_make -j4

echo ""
echo "Step 3: Sourcing environment..."
source devel/setup.bash

echo ""
echo "Step 4: Verifying node installation..."
ls -la devel/lib/perception_wego/ | grep traffic_light_detect_node.py
ls -la devel/lib/decision_wego/ | grep mission_traffic_light.py

echo ""
echo "=================================================="
echo "✓ Build Complete!"
echo ""
echo "To test the system:"
echo "  Terminal 1: roslaunch perception_wego perception_all.launch"
echo "  Terminal 2: roslaunch decision_wego decision_all.launch"
echo ""
echo "To verify traffic light detection:"
echo "  rostopic echo /webot/traffic_light/state"
echo ""
echo "To visualize:"
echo "  rqt_image_view /webot/traffic_light/image"
echo "  rqt_reconfigure   # Tune TrafficLight parameters"
echo "=================================================="

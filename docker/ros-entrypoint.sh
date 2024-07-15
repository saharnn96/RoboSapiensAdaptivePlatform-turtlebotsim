#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2
source /opt/ros/humble/setup.bash
source /opt/scan_relay_ws/install/setup.bash

# Source the base workspace, if built
if [ -f /opt/setup.bash ]
then
  source /opt/setup.bash
fi

# Execute the command passed into this entrypoint
exec "$@"
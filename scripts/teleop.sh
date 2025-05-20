#!/bin/bash

echo "🎮 Starting teleop control..."
docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    rostopic pub -r 10 /my_robot/cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
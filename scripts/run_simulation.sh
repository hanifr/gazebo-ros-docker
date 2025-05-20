#!/bin/bash

echo "🚀 Starting Gazebo server..."
# Check if containers are running, if not start them
if [ "$(docker ps -q -f name=gazebo_sim)" == "" ]; then
    echo "Starting Gazebo container..."
    docker-compose up -d gazebo
fi

if [ "$(docker ps -q -f name=ros_control)" == "" ]; then
    echo "Starting ROS container..."
    docker-compose up -d ros
fi

echo "⏳ Waiting for Gazebo to initialize..."
sleep 5

echo "🤖 Launching robot in simulation..."
docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    export GAZEBO_MASTER_URI=http://gazebo:11345 && \
    roslaunch my_robot_gazebo gazebo.launch"
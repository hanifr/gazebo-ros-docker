
#!/bin/bash

echo "🔍 Debugging Gazebo GUI on Mac..."

# Check if XQuartz is running
if ! ps -ef | grep -v grep | grep -q XQuartz; then
    echo "❌ XQuartz is not running! Please start XQuartz first."
    echo "   Open Applications/Utilities/XQuartz.app"
    exit 1
fi

# Use full path for xhost (common issue on Mac)
echo "🛠️ Setting X11 permissions..."
/opt/X11/bin/xhost + localhost 2>/dev/null || xhost + localhost 2>/dev/null

# Check Docker status
echo "🐳 Checking Docker containers..."
docker ps | grep -E 'gazebo|ros'

# Test X11 forwarding with a simple application
echo "🧪 Testing X11 forwarding with xeyes (if available)..."
docker-compose exec gazebo bash -c "apt-get update && apt-get install -y x11-apps && \
    export DISPLAY=host.docker.internal:0 && \
    xeyes" &
XEYES_PID=$!
sleep 5
kill $XEYES_PID 2>/dev/null

echo "📋 Checking environment inside container..."
docker-compose exec gazebo bash -c "echo DISPLAY=\$DISPLAY && \
    echo GAZEBO_MASTER_URI=\$GAZEBO_MASTER_URI && \
    ls -la /tmp/.X11-unix/ 2>/dev/null || echo 'X11 socket directory not found'"

echo "🧪 Trying Gazebo GUI with debug output..."
docker-compose exec gazebo bash -c "export DISPLAY=host.docker.internal:0 && \
    export QT_X11_NO_MITSHM=1 && \
    export LIBGL_ALWAYS_SOFTWARE=1 && \
    export MESA_GL_VERSION_OVERRIDE=3.3 && \
    export GAZEBO_MASTER_URI=http://gazebo:11345 && \
    echo 'Starting gzclient...' && \
    gzclient --verbose"

echo ""
echo "If Gazebo GUI still fails, you have these alternatives:"
echo "1. Use VNC server in the container (requires additional setup)"
echo "2. Use RVIZ for basic visualization instead of Gazebo GUI:"
echo "   docker-compose exec ros bash -c \"source /opt/ros/noetic/setup.bash && rosrun rviz rviz\""
echo "3. Continue using the teleop script to control the robot without visualization"

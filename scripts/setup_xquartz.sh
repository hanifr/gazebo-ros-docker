#!/bin/bash

# Check if XQuartz is installed
if ! command -v xquartz &> /dev/null; then
    echo "Installing XQuartz..."
    brew install --cask xquartz
else
    echo "XQuartz is already installed"
fi

# Configure XQuartz for remote connections
defaults write org.xquartz.X11 app_to_run /usr/bin/true
defaults write org.xquartz.X11 no_auth 1
defaults write org.xquartz.X11 nolisten_tcp 0

echo "✅ XQuartz configured successfully!"
echo "🔄 Please restart XQuartz now"
echo "🚀 After restarting, run: xhost + localhost"
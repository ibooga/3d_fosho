#!/bin/bash

# Script to run the ArcadeFPS game with proper Linux environment configuration
# This ensures window display works on Ubuntu Linux with OGRE Next 3.0

echo "Starting ArcadeFPS with Linux display optimization..."

# Force X11 for maximum compatibility
export SDL_VIDEODRIVER=x11
export GDK_BACKEND=x11

# Ensure DISPLAY is set
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

# Disable window manager ping timeout
export SDL_VIDEO_X11_NET_WM_PING=0

# For development: disable mouse grab for easier debugging
export SDL_VIDEO_X11_MOUSE_GRAB=0

# Check OpenGL capabilities
echo "Checking system OpenGL capabilities..."
glxinfo | grep "OpenGL version" || echo "glxinfo not available - install mesa-utils"

# Check for required libraries
echo "Checking required libraries..."
ldd ./build/ArcadeFPS 2>/dev/null | grep -E "(not found|libOgre|libGL)" || echo "Executable not found at ./build/ArcadeFPS"

# Run the game
if [ -f "./build/ArcadeFPS" ]; then
    echo "Launching ArcadeFPS..."
    ./build/ArcadeFPS
else
    echo "Error: ArcadeFPS executable not found at ./build/ArcadeFPS"
    echo "Please build the project first with: mkdir -p build && cd build && cmake .. && make"
    exit 1
fi
# ArcadeFPS

A simple first-person shooter style game inspired by 1980s arcade machines. It
uses **OGRE Next** for rendering and **Bullet** for physics. The controls are
shown on screen in an arcade-like fashion using OGRE's tray system.

## Requirements
- [OGRE Next 3.0.0](https://github.com/OGRECave/ogre-next/releases)
- [Bullet Physics 3.x](https://github.com/bulletphysics/bullet3)
- C++17 compatible compiler
- CMake 3.10 or newer

Make sure OGRE Next and Bullet are installed and visible to CMake via
`OGRE_DIR` and `BULLET_ROOT` (or system paths).

## Building
```bash
mkdir build
cd build
cmake ..
make
```

## Running
After building, run the produced `ArcadeFPS` executable. A small window will
appear with basic 3D graphics. Movement is controlled with **WASD**, jumping is
**Space**, and shooting is the left mouse button. Press **Esc** to quit.

## Running without a display server
To run the game on systems without a graphical display, start it under
`xvfb-run` which provides a virtual framebuffer:

```bash
xvfb-run -a -s "-screen 0 1024x768x24" ./build/ArcadeFPS
```

Ensure OGRE's media assets are installed and that GPU drivers are properly
configured.

This project is intended as a starting point for experimenting with OGRE Next
and Bullet. It displays 1980s inspired on-screen instructions and fires small
spheres as bullets using Bullet's physics simulation.

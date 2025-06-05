cmake_minimum_required(VERSION 3.10)
project(ArcadeFPS)

# Set C++ standard for OGRE Next 3.0 compatibility
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OGRE REQUIRED COMPONENTS Bites Overlay)
find_package(Bullet REQUIRED)
find_package(SDL2 REQUIRED)

# Remove OGRE's Bullet component to avoid conflicts
list(REMOVE_ITEM OGRE_LIBRARIES OgreBullet)

add_executable(ArcadeFPS
    src/main.cpp
    src/GameApp.cpp
    src/Player.cpp
    src/Weapon.cpp
)

target_include_directories(ArcadeFPS PRIVATE 
    ${OGRE_INCLUDE_DIRS} 
    ${BULLET_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(ArcadeFPS PRIVATE 
    ${OGRE_LIBRARIES} 
    ${BULLET_LIBRARIES}
    ${SDL2_LIBRARIES}
    pthread
    dl
)

# Add compiler flags for Linux compatibility
if(UNIX AND NOT APPLE)
    target_compile_definitions(ArcadeFPS PRIVATE LINUX_BUILD)
    target_link_libraries(ArcadeFPS PRIVATE X11 GL)
endif()


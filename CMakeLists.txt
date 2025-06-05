cmake_minimum_required(VERSION 3.10)
project(ArcadeFPS)

find_package(OGRE REQUIRED COMPONENTS Bites Overlay)
find_package(Bullet REQUIRED)

# Remove OGRE's Bullet component to avoid conflicts
list(REMOVE_ITEM OGRE_LIBRARIES OgreBullet)

add_executable(ArcadeFPS
    src/main.cpp
    src/GameApp.cpp
    src/Player.cpp
    src/Weapon.cpp
)

target_include_directories(ArcadeFPS PRIVATE ${OGRE_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS})
target_link_libraries(ArcadeFPS PRIVATE ${OGRE_LIBRARIES} ${BULLET_LIBRARIES})


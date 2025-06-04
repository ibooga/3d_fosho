cmake_minimum_required(VERSION 3.10)
project(ArcadeFPS)

find_package(OGRE 3.0 REQUIRED COMPONENTS Bites Overlay)
find_package(Bullet REQUIRED)

add_executable(ArcadeFPS
    src/main.cpp
    src/GameApp.cpp
    src/Player.cpp
)

target_include_directories(ArcadeFPS PRIVATE ${OGRE_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS})
target_link_libraries(ArcadeFPS PRIVATE ${OGRE_LIBRARIES} ${BULLET_LIBRARIES})


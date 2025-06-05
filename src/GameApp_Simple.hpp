#ifndef GAME_APP_SIMPLE_HPP
#define GAME_APP_SIMPLE_HPP

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreWindow.h>
#include <btBulletDynamicsCommon.h>
#include <vector>

// Simplified version without OgreBites for OGRE-Next compatibility

class GameApp
{
public:
    GameApp();
    ~GameApp();

    void initApp();
    void closeApp();
    Ogre::Root* getRoot() { return mRoot; }
    void runGameLoop();

private:
    void createScene();
    void loadLevel();
    void update(float deltaTime);

    Ogre::Root* mRoot;
    Ogre::Window* mWindow;
    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mCamera;
    Ogre::SceneNode* mCameraNode;
    
    btDiscreteDynamicsWorld* mDynamicsWorld;
    btAlignedObjectArray<btCollisionShape*> mCollisionShapes;
    
    // Game state
    float mRunTime;
    int mHealth;
    int mScore;
};

#endif // GAME_APP_SIMPLE_HPP
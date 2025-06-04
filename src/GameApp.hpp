#ifndef GAME_APP_HPP
#define GAME_APP_HPP

#include <OgreApplicationContext.h>
#include <OgreInput.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreOverlaySystem.h>
#include <OgreBitesConfigDialog.h>
#include <OgreTrays.h>

#include <btBulletDynamicsCommon.h>
#include <string>

class GameApp;

class InputHandler : public OgreBites::InputListener
{
public:
    InputHandler(GameApp* app);

    bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
    bool keyReleased(const OgreBites::KeyboardEvent& evt) override;
    bool mouseMoved(const OgreBites::MouseMotionEvent& evt) override;
    bool mousePressed(const OgreBites::MouseButtonEvent& evt) override;

    void update(float dt);

private:
    GameApp* mApp;
    Ogre::Vector3 mDirection;
    bool mJump;
};

class GameApp : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
    GameApp();
    ~GameApp();

    void setup() override;
    bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
    bool mousePressed(const OgreBites::MouseButtonEvent& evt) override;
    bool frameRenderingQueued(const Ogre::FrameEvent& evt) override;

private:
    void addStaticCube(const Ogre::Vector3& position, const Ogre::Vector3& scale);
    void loadLevel(const std::string& filename);
    void createBullet(const Ogre::Vector3& position, const Ogre::Quaternion& orient);

    btDiscreteDynamicsWorld* mDynamicsWorld;
    btBroadphaseInterface* mBroadphase;
    btCollisionDispatcher* mDispatcher;
    btSequentialImpulseConstraintSolver* mSolver;
    btDefaultCollisionConfiguration* mCollisionConfig;
    btAlignedObjectArray<btCollisionShape*> mCollisionShapes;
    Ogre::SceneNode* mCameraNode;
    Ogre::SceneManager* mSceneMgr;
    OgreBites::TrayManager* mTrayMgr;
    Ogre::OverlaySystem* mOverlaySystem;
    InputHandler* mInputHandler;
};

#endif // GAME_APP_HPP

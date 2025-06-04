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
};

#endif // GAME_APP_HPP

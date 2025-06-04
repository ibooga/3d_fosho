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
#include <vector>

#include "Weapon.hpp"

class GameApp;

struct Projectile
{
    Ogre::SceneNode* node;
    btRigidBody* body;
    int damage;
};

struct Target
{
    Ogre::SceneNode* node;
    btRigidBody* body;
    int health;
};

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

struct GameState
{
    int health;
    int ammo;
    int score;
    bool paused;
    bool gameOver;
    bool won;

    GameState()
        : health(100), ammo(20), score(0), paused(false), gameOver(false),
          won(false)
    {
    }
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

    void restartGame();

private:
    void addStaticCube(const Ogre::Vector3& position, const Ogre::Vector3& scale);
    void loadLevel(const std::string& filename);
    void createBullet(const Ogre::Vector3& position, const Ogre::Quaternion& orient);
    void checkProjectiles();

    void togglePause();
    void updateHUD();
    void setGameOver(bool won);

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

    std::vector<Projectile*> mProjectiles;
    std::vector<Target*> mTargets;

    Weapon* mWeapon;
    OgreBites::Label* mWeaponLabel;
};

#endif // GAME_APP_HPP

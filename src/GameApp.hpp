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

#include "Player.hpp"
#include "Weapon.hpp"

#include <vector>

class GameApp;

struct BulletProjectile
{
    Ogre::SceneNode* node;
    btRigidBody* body;
    float life;
};

class Enemy
{
public:
    Enemy(Ogre::SceneManager* sceneMgr, btDiscreteDynamicsWorld* world,
          btAlignedObjectArray<btCollisionShape*>& collisionShapes,
          const Ogre::Vector3& position);
    ~Enemy();

    void update(float dt, const Ogre::Vector3& playerPos);
    void takeDamage(int amount) { mHealth -= amount; }
    bool isDead() const { return mHealth <= 0; }

    Ogre::SceneNode* getNode() const { return mNode; }
    btRigidBody* getBody() const { return mBody; }

private:
    Ogre::SceneNode* mNode;
    btRigidBody* mBody;
    int mHealth;
    Ogre::Vector3 mSpawnPos;
    Ogre::Vector3 mPatrolDir;
    float mPatrolDistance;
    float mTraveled;
    enum class State { Patrol, Chase, Attack } mState;
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

    Ogre::SceneNode* getCameraNode() const { return mCameraNode; }
    Player* getPlayer() const { return mPlayer; }
    void restartGame();

private:
    void addStaticCube(const Ogre::Vector3& position, const Ogre::Vector3& scale);
    void loadLevel(const std::string& filename);
    void createBullet(const Ogre::Vector3& position, const Ogre::Quaternion& orient);
    void checkProjectiles();

    void togglePause();
    void updateHUD();
    void setGameOver(bool won);

    void spawnEnemy(const Ogre::Vector3& position);

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

    Player* mPlayer;

    std::vector<BulletProjectile*> mBullets;
    std::vector<Enemy*> mEnemies;

    GameState mGameState;

    OgreBites::Label* mCrosshair{nullptr};
    OgreBites::ProgressBar* mHealthBar{nullptr};
    OgreBites::Label* mScoreLabel{nullptr};
    OgreBites::Label* mWeaponLabel{nullptr};
    Weapon* mWeapon{nullptr};

};

#endif // GAME_APP_HPP

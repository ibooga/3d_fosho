#include "GameApp.hpp"
#include "Weapon.hpp"

#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreRoot.h>
#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreStringConverter.h>
#include <fstream>
#include <sstream>


InputHandler::InputHandler(GameApp* app)
    : mApp(app), mDirection(Ogre::Vector3::ZERO), mJump(false)
{
}

bool InputHandler::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    if (evt.keysym.sym == OgreBites::SDLK_w)
        mDirection.z = -1;
    else if (evt.keysym.sym == OgreBites::SDLK_s)
        mDirection.z = 1;
    else if (evt.keysym.sym == OgreBites::SDLK_a)
        mDirection.x = -1;
    else if (evt.keysym.sym == OgreBites::SDLK_d)
        mDirection.x = 1;
    else if (evt.keysym.sym == OgreBites::SDLK_SPACE)
        mJump = true;
    return true;
}

bool InputHandler::keyReleased(const OgreBites::KeyboardEvent& evt)
{
    if (evt.keysym.sym == OgreBites::SDLK_w || evt.keysym.sym == OgreBites::SDLK_s)
        mDirection.z = 0;
    if (evt.keysym.sym == OgreBites::SDLK_a || evt.keysym.sym == OgreBites::SDLK_d)
        mDirection.x = 0;
    return true;
}

bool InputHandler::mouseMoved(const OgreBites::MouseMotionEvent& evt)
{
    const float sensitivity = 0.1f;
    mApp->getPlayer()->getNode()->yaw(Ogre::Degree(-evt.xrel * sensitivity), Ogre::Node::TS_WORLD);
    mApp->getCameraNode()->pitch(Ogre::Degree(-evt.yrel * sensitivity));
    return true;
}

bool InputHandler::mousePressed(const OgreBites::MouseButtonEvent& evt)
{
    if (evt.button == OgreBites::BUTTON_LEFT)
    {
        Ogre::Vector3 pos = mApp->mCameraNode->getPosition();
        Ogre::Quaternion orient = mApp->mCameraNode->getOrientation();
        if (mApp->mWeapon)
            mApp->mWeapon->fire(pos + orient * Ogre::Vector3(0,0,-1), orient);
        if (mApp->mWeaponLabel)
        {
            mApp->mWeaponLabel->setCaption(mApp->mWeapon->getName() + " - " +
                                           std::to_string(mApp->mWeapon->getAmmo()));
        }
    }
    return true;
}

void InputHandler::update(float dt)
{
    const float force = 10.0f;
    Ogre::Vector3 worldDir = mApp->getPlayer()->getNode()->getOrientation() * mDirection;
    mApp->getPlayer()->applyMovement(worldDir, force);
    if (mApp->mGameState.paused || mApp->mGameState.gameOver)
        return;
    const float speed = 5.0f;
    Ogre::Vector3 disp = mDirection * speed * dt;
    mApp->mCameraNode->translate(mApp->mCameraNode->getOrientation() * disp, Ogre::Node::TS_WORLD);
  
    if (mJump)
    {
        mApp->getPlayer()->jump(4.0f);
        mJump = false;
    }
    if (mApp->mWeapon)
        mApp->mWeapon->update(dt);
}

// ---------------------------------------------------------------------
Enemy::Enemy(Ogre::SceneManager* sceneMgr, btDiscreteDynamicsWorld* world,
             btAlignedObjectArray<btCollisionShape*>& collisionShapes,
             const Ogre::Vector3& position)
    : mNode(nullptr), mBody(nullptr), mHealth(3), mSpawnPos(position),
      mPatrolDir(Ogre::Vector3::UNIT_X), mPatrolDistance(5.0f), mTraveled(0.0f),
      mState(State::Patrol)
{
    Ogre::Entity* ent = sceneMgr->createEntity(Ogre::SceneManager::PT_CUBE);
    mNode = sceneMgr->getRootSceneNode()->createChildSceneNode(position);
    mNode->setScale(0.5f, 0.5f, 0.5f);
    mNode->attachObject(ent);

    btCollisionShape* shape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
    collisionShapes.push_back(shape);
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(position.x, position.y, position.z));
    btScalar mass = 1.0f;
    btVector3 inertia(0,0,0);
    shape->calculateLocalInertia(mass, inertia);
    btDefaultMotionState* motion = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, inertia);
    mBody = new btRigidBody(info);
    mBody->setAngularFactor(0);
    world->addRigidBody(mBody);
}

Enemy::~Enemy()
{
    if (mNode)
        mNode->getParentSceneNode()->removeAndDestroyChild(mNode);
    // body removed by dynamics world outside
}

void Enemy::update(float dt, const Ogre::Vector3& playerPos)
{
    btTransform trans;
    mBody->getMotionState()->getWorldTransform(trans);
    Ogre::Vector3 pos(trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z());

    float distanceToPlayer = pos.distance(playerPos);
    if (distanceToPlayer < 2.0f)
        mState = State::Attack;
    else if (distanceToPlayer < 10.0f)
        mState = State::Chase;
    else if (distanceToPlayer > 12.0f)
        mState = State::Patrol;

    Ogre::Vector3 velocity = Ogre::Vector3::ZERO;
    const float speed = 2.0f;
    switch (mState)
    {
    case State::Patrol:
        velocity = mPatrolDir * speed;
        mTraveled += dt * speed;
        if (mTraveled > mPatrolDistance)
        {
            mTraveled = 0.0f;
            mPatrolDir = -mPatrolDir;
        }
        break;
    case State::Chase:
        velocity = (playerPos - pos).normalisedCopy() * speed * 1.5f;
        break;
    case State::Attack:
        velocity = Ogre::Vector3::ZERO;
        break;
    }

    mBody->setLinearVelocity(btVector3(velocity.x, velocity.y, velocity.z));

    // sync scene node
    mNode->setPosition(pos);
}

GameApp::GameApp() : OgreBites::ApplicationContext("ArcadeFPS"),
                     mDynamicsWorld(nullptr),
                     mBroadphase(nullptr),
                     mDispatcher(nullptr),
                     mSolver(nullptr),
                     mCollisionConfig(nullptr),
                     mCameraNode(nullptr),
                     mSceneMgr(nullptr),
                     mTrayMgr(nullptr),
                     mOverlaySystem(nullptr),
                     mInputHandler(nullptr),
                     mPlayer(nullptr),
                     mWeapon(nullptr),
                     mWeaponLabel(nullptr),
                     mCrosshair(nullptr),
                     mHealthBar(nullptr),
                     mScoreLabel(nullptr)
{
}

GameApp::~GameApp()
{
    if (mTrayMgr)
    {
        delete mTrayMgr;
        mTrayMgr = nullptr;
    }

    if (mSceneMgr && mOverlaySystem)
        mSceneMgr->removeRenderQueueListener(mOverlaySystem);
    delete mOverlaySystem;
    mOverlaySystem = nullptr;

    delete mWeaponLabel;
    mWeaponLabel = nullptr;
    delete mWeapon;
    mWeapon = nullptr;

    for (int i = 0; i < mCollisionShapes.size(); ++i)
        delete mCollisionShapes[i];
    mCollisionShapes.clear();


    delete mDynamicsWorld;
    mDynamicsWorld = nullptr;
    delete mSolver;
    mSolver = nullptr;
    delete mBroadphase;
    mBroadphase = nullptr;
    delete mDispatcher;
    mDispatcher = nullptr;
    delete mCollisionConfig;
    mCollisionConfig = nullptr;


    delete mPlayer;
    mPlayer = nullptr;
    for (auto bullet : mBullets)
    {
        mDynamicsWorld->removeRigidBody(bullet->body);
        delete bullet->body->getMotionState();
        delete bullet->body;
        if (bullet->node)
            bullet->node->getParentSceneNode()->removeAndDestroyChild(bullet->node);
        delete bullet;
    }
    mBullets.clear();

    for (auto enemy : mEnemies)
    {
        mDynamicsWorld->removeRigidBody(enemy->getBody());
        delete enemy->getBody()->getMotionState();
        delete enemy->getBody();
        delete enemy;
    }
    mEnemies.clear();

    delete mInputHandler;
    mInputHandler = nullptr;
}

void GameApp::setup()
{
    OgreBites::ApplicationContext::setup();
    addInputListener(this);

    // Overlay system for 2D elements
    mOverlaySystem = new Ogre::OverlaySystem();
    mSceneMgr = getRoot()->createSceneManager();
    mSceneMgr->addRenderQueueListener(mOverlaySystem);

    // tray manager for 80s style controls
    mTrayMgr = new OgreBites::TrayManager("HUD", getRenderWindow());
    mTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
    mTrayMgr->showCursor();
    addInputListener(mTrayMgr);

    mTrayMgr->createLabel(OgreBites::TL_TOP, "Controls", "INSERT COIN - WASD to Move, SPACE to Jump, LMB to Shoot", 400);
    mCrosshair = mTrayMgr->createLabel(OgreBites::TL_CENTER, "Crosshair", "+", 50);
    mHealthBar = mTrayMgr->createProgressBar(OgreBites::TL_BOTTOM, "Health", 200, 20);
    mHealthBar->setProgress(1.0f);
    mScoreLabel = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "Score", "Score: 0", 120);

    // Bullet physics setup
    mCollisionConfig = new btDefaultCollisionConfiguration();
    mDispatcher = new btCollisionDispatcher(mCollisionConfig);
    mBroadphase = new btDbvtBroadphase();
    mSolver = new btSequentialImpulseConstraintSolver();
    mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfig);
    mDynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

    // create player
    mPlayer = new Player(mSceneMgr, mDynamicsWorld, mCollisionShapes);

    // camera
    Ogre::Camera* cam = mSceneMgr->createCamera("MainCam");
    cam->setNearClipDistance(0.1f);
    cam->setAutoAspectRatio(true);
    mCameraNode = mPlayer->getNode()->createChildSceneNode(Ogre::Vector3(0, 0.5f, 0));
    mCameraNode->attachObject(cam);
    getRenderWindow()->addViewport(cam);

    mInputHandler = new InputHandler(this);
    addInputListener(mInputHandler);

    mWeapon = new Pistol(this);
    mWeaponLabel = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "Weapon", mWeapon->getName() + " - " + std::to_string(mWeapon->getAmmo()), 150);
    updateHUD();

    // lighting
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
    mSceneMgr->createLight()->setPosition(20, 80, 50);

    // floor
    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
    Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                  plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
    Ogre::Entity* groundEntity = mSceneMgr->createEntity("ground");
    groundEntity->setCastShadows(false);
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(groundEntity);

    // ground plane in Bullet
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0), 0);
    mCollisionShapes.push_back(groundShape);
    btDefaultMotionState* groundMotion = new btDefaultMotionState();
    btRigidBody::btRigidBodyConstructionInfo groundInfo(0.0f, groundMotion, groundShape);
    btRigidBody* groundBody = new btRigidBody(groundInfo);
    mDynamicsWorld->addRigidBody(groundBody);

    loadLevel("level.txt");

    spawnEnemy(Ogre::Vector3(5, 0.5f, -5));
    spawnEnemy(Ogre::Vector3(-5, 0.5f, -5));
    spawnEnemy(Ogre::Vector3(0, 0.5f, 5));
}

bool GameApp::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
    {
        getRoot()->queueEndRendering();
    }
    else if (evt.keysym.sym == OgreBites::SDLK_p)
    {
        if (!mGameState.gameOver)
            togglePause();
    }
    else if (evt.keysym.sym == OgreBites::SDLK_r)
    {
        if (mGameState.gameOver)
            restartGame();
    }
    return true;
}

bool GameApp::mousePressed(const OgreBites::MouseButtonEvent& evt)
{

    return true;
}

bool GameApp::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if (mDynamicsWorld)
        mDynamicsWorld->stepSimulation(evt.timeSinceLastFrame);
    if (mPlayer)
        mPlayer->update();
    if (mInputHandler)
        mInputHandler->update(evt.timeSinceLastFrame);

    Ogre::Vector3 playerPos = mCameraNode->getPosition();
    for (auto enemy : mEnemies)
        enemy->update(evt.timeSinceLastFrame, playerPos);

    // update bullets and check collisions
    for (auto it = mBullets.begin(); it != mBullets.end(); )
    {
        BulletProjectile* proj = *it;
        proj->life -= evt.timeSinceLastFrame;
        btTransform trans;
        proj->body->getMotionState()->getWorldTransform(trans);
        Ogre::Vector3 pos(trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z());
        proj->node->setPosition(pos);

        bool remove = proj->life <= 0.0f;
        for (auto enemy : mEnemies)
        {
            btTransform et;
            enemy->getBody()->getMotionState()->getWorldTransform(et);
            Ogre::Vector3 epos(et.getOrigin().x(), et.getOrigin().y(), et.getOrigin().z());
            if (pos.distance(epos) < 1.0f)
            {
                enemy->takeDamage(1);
                remove = true;
                break;
            }
        }
        if (remove)
        {
            mDynamicsWorld->removeRigidBody(proj->body);
            delete proj->body->getMotionState();
            delete proj->body;
            if (proj->node)
                proj->node->getParentSceneNode()->removeAndDestroyChild(proj->node);
            delete proj;
            it = mBullets.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // remove dead enemies
    for (auto it = mEnemies.begin(); it != mEnemies.end(); )
    {
        Enemy* e = *it;
        if (e->isDead())
        {
            mDynamicsWorld->removeRigidBody(e->getBody());
            delete e->getBody()->getMotionState();
            delete e->getBody();
            delete e;
            it = mEnemies.erase(it);
        }
        else
        {
            ++it;
        }
    }

    updateHUD();
    return true;
}

void GameApp::createBullet(const Ogre::Vector3& position, const Ogre::Quaternion& orient)
{
    Ogre::Entity* ball = mSceneMgr->createEntity(Ogre::SceneManager::PT_SPHERE);
    Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode(position, orient);
    node->setScale(0.1f, 0.1f, 0.1f);
    node->attachObject(ball);

    btCollisionShape* sphereShape = new btSphereShape(0.5f);
    mCollisionShapes.push_back(sphereShape);
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(position.x, position.y, position.z));
    btScalar mass = 1.0f;
    btVector3 inertia(0,0,0);
    sphereShape->calculateLocalInertia(mass, inertia);
    btDefaultMotionState* motion = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motion, sphereShape, inertia);
    btRigidBody* body = new btRigidBody(info);

    Ogre::Vector3 forward = orient * Ogre::Vector3::NEGATIVE_UNIT_Z;
    body->setLinearVelocity(btVector3(forward.x, forward.y, forward.z) * 25.0f);

    Projectile* basicProj = new Projectile();
    basicProj->node = node;
    basicProj->body = body;
    basicProj->damage = 10;

    mDynamicsWorld->addRigidBody(body);

    BulletProjectile* bulletProj = new BulletProjectile();
    bulletProj->node = node;
    bulletProj->body = body;
    bulletProj->life = 3.0f; // seconds
    mBullets.push_back(bulletProj);
}

void GameApp::addStaticCube(const Ogre::Vector3& position, const Ogre::Vector3& scale)
{
    Ogre::Entity* ent = mSceneMgr->createEntity(Ogre::SceneManager::PT_CUBE);
    Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode(position);
    node->setScale(scale);
    node->attachObject(ent);

    btCollisionShape* shape = new btBoxShape(btVector3(scale.x, scale.y, scale.z));
    mCollisionShapes.push_back(shape);
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(position.x, position.y, position.z));
    btDefaultMotionState* motion = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(0.0f, motion, shape);
    btRigidBody* body = new btRigidBody(info);
    mDynamicsWorld->addRigidBody(body);
}

void GameApp::loadLevel(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
        return;

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;

        std::istringstream iss(line);
        std::string type;
        float x, y, z, sx, sy, sz;
        if (!(iss >> type >> x >> y >> z >> sx >> sy >> sz))
            continue;

        if (type == "wall" || type == "obstacle")
            addStaticCube(Ogre::Vector3(x, y, z), Ogre::Vector3(sx, sy, sz));
    }
}

void GameApp::spawnEnemy(const Ogre::Vector3& position)
{
    Enemy* e = new Enemy(mSceneMgr, mDynamicsWorld, mCollisionShapes, position);
    mEnemies.push_back(e);

}

void GameApp::updateHUD()
{
    if (mHealthBar)
        mHealthBar->setProgress(static_cast<Ogre::Real>(mGameState.health) / 100.0f);
    if (mScoreLabel)
        mScoreLabel->setCaption("Score: " + Ogre::StringConverter::toString(mGameState.score));
    if (mWeaponLabel && mWeapon)
        mWeaponLabel->setCaption(mWeapon->getName() + " - " + std::to_string(mWeapon->getAmmo()));
}

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
#include <cstdlib>
#include <cstdio>


InputHandler::InputHandler(GameApp* app)
    : mApp(app), mDirection(Ogre::Vector3::ZERO), mJump(false)
{
}

bool InputHandler::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    if (evt.keysym.sym == 'w')
        mDirection.z = -1;
    else if (evt.keysym.sym == 's')
        mDirection.z = 1;
    else if (evt.keysym.sym == 'a')
        mDirection.x = -1;
    else if (evt.keysym.sym == 'd')
        mDirection.x = 1;
    else if (evt.keysym.sym == OgreBites::SDLK_SPACE)
        mJump = true;
    return true;
}

bool InputHandler::keyReleased(const OgreBites::KeyboardEvent& evt)
{
    if (evt.keysym.sym == 'w' || evt.keysym.sym == 's')
        mDirection.z = 0;
    if (evt.keysym.sym == 'a' || evt.keysym.sym == 'd')
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
    // Mouse button handling moved to GameApp
    return false;
}

void InputHandler::update(float dt)
{
    const float force = 10.0f;
    Ogre::Vector3 worldDir = mApp->getPlayer()->getNode()->getOrientation() * mDirection;
    mApp->getPlayer()->applyMovement(worldDir, force);
    if (mApp->getGameState().paused || mApp->getGameState().gameOver)
        return;
    const float speed = 5.0f;
    Ogre::Vector3 disp = mDirection * speed * dt;
    mApp->getCameraNode()->translate(mApp->getCameraNode()->getOrientation() * disp, Ogre::Node::TS_WORLD);
  
    if (mJump)
    {
        mApp->getPlayer()->jump(4.0f);
        mJump = false;
    }
    if (mApp->getWeapon())
        mApp->getWeapon()->update(dt);
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
    mBody->setUserPointer(this);
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

void GameApp::configureLinuxEnvironment()
{
    // Set SDL hints for Linux compatibility
    SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_PING, "0");
    
    // Prefer X11 for stability over Wayland
    if (!getenv("SDL_VIDEODRIVER"))
    {
        SDL_SetHint(SDL_HINT_VIDEODRIVER, "x11");
        setenv("SDL_VIDEODRIVER", "x11", 1);
        setenv("GDK_BACKEND", "x11", 1);
    }
    
    // Ensure DISPLAY is set for headless environments
    if (!getenv("DISPLAY"))
    {
        setenv("DISPLAY", ":0", 1);
    }
    
    // Log the configuration
    const char* sessionType = getenv("XDG_SESSION_TYPE");
    const char* videoDriver = getenv("SDL_VIDEODRIVER");
    const char* display = getenv("DISPLAY");
    
    printf("Linux Environment Configuration:\n");
    printf("  XDG_SESSION_TYPE: %s\n", sessionType ? sessionType : "not set");
    printf("  SDL_VIDEODRIVER: %s\n", videoDriver ? videoDriver : "not set");
    printf("  DISPLAY: %s\n", display ? display : "not set");
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
    // Configure Linux environment before OGRE initialization
    configureLinuxEnvironment();
}

int GameApp::go()
{
    try {
        // Ensure SDL video subsystem is available
        if (SDL_WasInit(SDL_INIT_VIDEO) == 0)
        {
            if (SDL_InitSubSystem(SDL_INIT_VIDEO) != 0)
            {
                Ogre::LogManager::getSingleton().logMessage("Failed to initialize SDL video subsystem: " + std::string(SDL_GetError()));
                return 1;
            }
        }
        
        initApp();
        
        // Verify render window was created successfully
        if (!getRenderWindow())
        {
            Ogre::LogManager::getSingleton().logMessage("Failed to create render window");
            return 1;
        }
        
        getRoot()->startRendering();
        closeApp();
        return 0;
    } catch (const Ogre::Exception& e) {
        Ogre::LogManager::getSingleton().logMessage("OGRE Exception: " + e.getFullDescription());
        return 1;
    } catch (const std::exception& e) {
        Ogre::LogManager::getSingleton().logMessage("Standard Exception: " + std::string(e.what()));
        return 1;
    } catch (...) {
        Ogre::LogManager::getSingleton().logMessage("Unknown exception occurred");
        return 1;
    }
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
    
    // Only delete overlay system if we created it ourselves
    if (mOverlaySystem != getOverlaySystem())
    {
        delete mOverlaySystem;
    }
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
    // Add GameApp as the initial input listener
    addInputListener(this);
    
    // Enable verbose OGRE logging for debugging
    if (Ogre::LogManager::getSingleton().getDefaultLog())
    {
        Ogre::LogManager::getSingleton().getDefaultLog()->setLogDetail(Ogre::LL_BOREME);
    }
    
    // Ensure all resource groups are initialized
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    // Create scene manager first
    mSceneMgr = getRoot()->createSceneManager();
    
    // Overlay system for 2D elements - use ApplicationContext's overlay system if available
    mOverlaySystem = getOverlaySystem();
    if (!mOverlaySystem)
    {
        mOverlaySystem = new Ogre::OverlaySystem();
    }
    mSceneMgr->addRenderQueueListener(mOverlaySystem);

    // tray manager for 80s style controls
    mTrayMgr = new OgreBites::TrayManager("HUD", getRenderWindow());
    mTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
    mTrayMgr->showCursor();
    addInputListener(mTrayMgr);

    mTrayMgr->createLabel(OgreBites::TL_TOP, "Controls", "INSERT COIN - WASD to Move, SPACE to Jump, LMB to Shoot", 400);
    mCrosshair = mTrayMgr->createLabel(OgreBites::TL_CENTER, "Crosshair", "+", 50);
    mHealthBar = mTrayMgr->createProgressBar(OgreBites::TL_BOTTOM, "Health", "Health", 200, 20);
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

    // Configure mouse input before creating the input handler
    getRenderWindow()->setMouseGrab(true);
    getRenderWindow()->setMouseRelative(true);
    getRenderWindow()->setMouseVisible(false);

    mInputHandler = new InputHandler(this);
    // listener added at end of setup

    mWeapon = new Pistol(this);
    mWeaponLabel = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "Weapon", mWeapon->getName() + " - " + std::to_string(mWeapon->getAmmo()), 150);
    updateHUD();

    // lighting
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
    Ogre::Light* light = mSceneMgr->createLight();
    Ogre::SceneNode* lightNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    lightNode->setPosition(20, 80, 50);

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

    // Add input handler after all systems are initialized
    addInputListener(mInputHandler);
}

bool GameApp::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
    {
        getRoot()->queueEndRendering();
    }
    else if (evt.keysym.sym == 'p')
    {
        if (!mGameState.gameOver)
            togglePause();
    }
    else if (evt.keysym.sym == 'r')
    {
        if (mGameState.gameOver)
            restartGame();
    }
    return true;
}

bool GameApp::mousePressed(const OgreBites::MouseButtonEvent& evt)
{
    Ogre::LogManager::getSingleton().logMessage(
        "GameApp::mousePressed - Button: " + std::to_string(evt.button) +
        " at (" + std::to_string(evt.x) + ", " + std::to_string(evt.y) + ")");

    if (evt.button == OgreBites::BUTTON_LEFT)
    {
        if (mWeapon && mCameraNode)
        {
            if (mWeapon->canFire())
            {
                Ogre::Vector3 camPos = mCameraNode->_getDerivedPosition();
                Ogre::Quaternion camOrient = mCameraNode->_getDerivedOrientation();
                Ogre::Vector3 firePos = camPos + camOrient * Ogre::Vector3(0, -0.1f, -0.5f);

                Ogre::LogManager::getSingleton().logMessage(
                    "Firing weapon! Ammo: " + std::to_string(mWeapon->getAmmo()));

                mWeapon->fire(firePos, camOrient);
            }
            else
            {
                Ogre::LogManager::getSingleton().logMessage(
                    "Cannot fire - Cooldown or no ammo. Ammo: " +
                    std::to_string(mWeapon->getAmmo()));
            }
        }
        else
        {
            Ogre::LogManager::getSingleton().logMessage(
                "Cannot fire - Weapon or camera node is null");
        }
    }

    return true;
}

bool GameApp::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    static bool debugPrinted = false;
    if (!debugPrinted)
    {
        debugInputSystem();
        debugPrinted = true;
    }

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

        struct BulletCallback : public btCollisionWorld::ContactResultCallback
        {
            BulletProjectile* p;
            GameApp* app;
            BulletCallback(BulletProjectile* bp, GameApp* a) : p(bp), app(a) {}
            btScalar addSingleResult(btManifoldPoint&,
                                     const btCollisionObjectWrapper* col0,int,int,
                                     const btCollisionObjectWrapper* col1,int,int) override
            {
                const btCollisionObject* other =
                    col0->getCollisionObject() == p->body ? col1->getCollisionObject()
                                                         : col0->getCollisionObject();
                Enemy* enemy = static_cast<Enemy*>(other->getUserPointer());
                if (enemy)
                    enemy->takeDamage(p->damage);
                p->remove = true;
                return 0;
            }
        };

        BulletCallback cb(proj, this);
        mDynamicsWorld->contactTest(proj->body, cb);

        bool remove = proj->life <= 0.0f || proj->remove;
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

    BulletProjectile* proj = new BulletProjectile();
    proj->node = node;
    proj->body = body;
    proj->life = 3.0f; // seconds
    proj->damage = 10;
    body->setUserPointer(proj);
    mDynamicsWorld->addRigidBody(body);
    mBullets.push_back(proj);
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

void GameApp::togglePause()
{
    mGameState.paused = !mGameState.paused;
}

void GameApp::restartGame()
{
    mGameState = GameState();
}

void GameApp::loadLevel(const std::string& filename)
{
    // Simple level loading from level.txt
    std::ifstream file(filename);
    if (!file.is_open()) return;
    
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string type;
        float x, y, z, sx, sy, sz;
        
        if (iss >> type >> x >> y >> z >> sx >> sy >> sz)
        {
            if (type == "wall" || type == "obstacle")
            {
                addStaticCube(Ogre::Vector3(x, y, z), Ogre::Vector3(sx, sy, sz));
            }
        }
    }
}

void GameApp::addStaticCube(const Ogre::Vector3& position, const Ogre::Vector3& scale)
{
    Ogre::Entity* cubeEntity = mSceneMgr->createEntity(Ogre::SceneManager::PT_CUBE);
    Ogre::SceneNode* cubeNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(position);
    cubeNode->setScale(scale);
    cubeNode->attachObject(cubeEntity);

    // Add physics body
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

void GameApp::debugInputSystem()
{
    Ogre::LogManager::getSingleton().logMessage("=== Input System Debug ===");
    Ogre::LogManager::getSingleton().logMessage("SDL Video Driver: " +
        std::string(SDL_GetCurrentVideoDriver()));

    SDL_Window* sdlWindow = SDL_GetWindowFromID(
        getRenderWindow()->getCustomAttribute("SDL_WINDOW_ID"));
    if (sdlWindow)
    {
        Ogre::LogManager::getSingleton().logMessage("SDL Window found");
        Ogre::LogManager::getSingleton().logMessage("Mouse grab: " +
            std::string(SDL_GetWindowGrab(sdlWindow) ? "true" : "false"));
        Ogre::LogManager::getSingleton().logMessage("Relative mouse: " +
            std::string(SDL_GetRelativeMouseMode() ? "true" : "false"));
    }

    if (mWeapon)
    {
        Ogre::LogManager::getSingleton().logMessage("Weapon: " + mWeapon->getName());
        Ogre::LogManager::getSingleton().logMessage("Ammo: " +
            std::to_string(mWeapon->getAmmo()));
        Ogre::LogManager::getSingleton().logMessage("Can fire: " +
            std::string(mWeapon->canFire() ? "yes" : "no"));
    }
}

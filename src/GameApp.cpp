#include "GameApp.hpp"

#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreRoot.h>
#include <OgreVector3.h>
#include <fstream>

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
    mApp->mCameraNode->yaw(Ogre::Degree(-evt.xrel * sensitivity), Ogre::Node::TS_WORLD);
    mApp->mCameraNode->pitch(Ogre::Degree(-evt.yrel * sensitivity));
    return true;
}

bool InputHandler::mousePressed(const OgreBites::MouseButtonEvent& evt)
{
    return true;
}

void InputHandler::update(float dt)
{
    const float speed = 5.0f;
    Ogre::Vector3 disp = mDirection * speed * dt;
    mApp->mCameraNode->translate(mApp->mCameraNode->getOrientation() * disp, Ogre::Node::TS_WORLD);
    if (mJump)
    {
        mApp->mCameraNode->translate(0, 3.0f * dt, 0, Ogre::Node::TS_WORLD);
        mJump = false;
    }
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
                     mInputHandler(nullptr)
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

    delete mInputHandler;
    mInputHandler = nullptr;

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

    // camera
    Ogre::Camera* cam = mSceneMgr->createCamera("MainCam");
    cam->setNearClipDistance(0.1f);
    cam->setAutoAspectRatio(true);
    mCameraNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    mCameraNode->attachObject(cam);
    getRenderWindow()->addViewport(cam);

    mInputHandler = new InputHandler(this);
    addInputListener(mInputHandler);

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

    // Bullet physics setup
    mCollisionConfig = new btDefaultCollisionConfiguration();
    mDispatcher = new btCollisionDispatcher(mCollisionConfig);
    mBroadphase = new btDbvtBroadphase();
    mSolver = new btSequentialImpulseConstraintSolver();
    mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfig);
    mDynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

    // ground plane in Bullet
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0), 0);
    mCollisionShapes.push_back(groundShape);
    btDefaultMotionState* groundMotion = new btDefaultMotionState();
    btRigidBody::btRigidBodyConstructionInfo groundInfo(0.0f, groundMotion, groundShape);
    btRigidBody* groundBody = new btRigidBody(groundInfo);
    mDynamicsWorld->addRigidBody(groundBody);

    loadLevel("level.txt");
}

bool GameApp::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
    {
        getRoot()->queueEndRendering();
    }
    return true;
}

bool GameApp::mousePressed(const OgreBites::MouseButtonEvent& evt)
{
    if (evt.button == OgreBites::BUTTON_LEFT)
    {
        Ogre::Vector3 pos = mCameraNode->getPosition();
        Ogre::Quaternion orient = mCameraNode->getOrientation();
        createBullet(pos + orient * Ogre::Vector3(0,0,-1), orient);
    }
    return true;
}

bool GameApp::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if (mDynamicsWorld)
        mDynamicsWorld->stepSimulation(evt.timeSinceLastFrame);
    if (mInputHandler)
        mInputHandler->update(evt.timeSinceLastFrame);
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
    mDynamicsWorld->addRigidBody(body);
}

void GameApp::addStaticCube(const Ogre::Vector3& position, const Ogre::Vector3& scale)
{
    Ogre::Entity* cube = mSceneMgr->createEntity("Cube.mesh");
    Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode(position);
    node->setScale(scale);
    node->attachObject(cube);

    btVector3 halfExtents(scale.x * 50.0f, scale.y * 50.0f, scale.z * 50.0f);
    btCollisionShape* box = new btBoxShape(halfExtents);
    mCollisionShapes.push_back(box);
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(position.x, position.y, position.z));
    btDefaultMotionState* motion = new btDefaultMotionState(tr);
    btRigidBody::btRigidBodyConstructionInfo info(0.0f, motion, box);
    btRigidBody* body = new btRigidBody(info);
    mDynamicsWorld->addRigidBody(body);
}

void GameApp::loadLevel(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
        return;

    std::string type;
    float x, y, z, sx, sy, sz;
    while (file >> type >> x >> y >> z >> sx >> sy >> sz)
    {
        Ogre::Vector3 pos(x, y, z);
        Ogre::Vector3 scale(sx, sy, sz);
        if (type == "wall" || type == "obstacle" || type == "item")
            addStaticCube(pos, scale);
    }
}

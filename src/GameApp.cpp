#include "GameApp.hpp"

#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreRoot.h>

GameApp::GameApp() : OgreBites::ApplicationContext("ArcadeFPS"),
                     mDynamicsWorld(nullptr),
                     mBroadphase(nullptr),
                     mDispatcher(nullptr),
                     mSolver(nullptr),
                     mCollisionConfig(nullptr),
                     mCameraNode(nullptr),
                     mSceneMgr(nullptr),
                     mTrayMgr(nullptr),
                     mOverlaySystem(nullptr)
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

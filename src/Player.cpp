#include "Player.hpp"

#include <cmath>
#include <OgreSceneNode.h>

Player::Player(Ogre::SceneManager* sceneMgr,
               btDiscreteDynamicsWorld* world,
               btAlignedObjectArray<btCollisionShape*>& collisionShapes)
    : mWorld(world)
{
    mNode = sceneMgr->getRootSceneNode()->createChildSceneNode();

    btCollisionShape* shape = new btCapsuleShape(0.5f, 1.0f);
    collisionShapes.push_back(shape);

    btTransform start;
    start.setIdentity();
    start.setOrigin(btVector3(0, 1.0f, 0));

    btScalar mass = 1.0f;
    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(mass, inertia);
    btDefaultMotionState* motion = new btDefaultMotionState(start);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, inertia);
    mBody = new btRigidBody(info);
    mBody->setAngularFactor(0);

    mWorld->addRigidBody(mBody);
}

Player::~Player()
{
    mWorld->removeRigidBody(mBody);
    delete mBody->getMotionState();
    delete mBody;
}

void Player::update()
{
    btTransform trans;
    mBody->getMotionState()->getWorldTransform(trans);
    const btVector3& p = trans.getOrigin();
    mNode->setPosition(Ogre::Vector3(p.x(), p.y(), p.z()));
}

void Player::applyMovement(const Ogre::Vector3& dir, float force)
{
    if (dir == Ogre::Vector3::ZERO)
        return;
    btVector3 f(dir.x, dir.y, dir.z);
    f *= force;
    mBody->applyCentralForce(f);
}

void Player::jump(float impulse)
{
    if (onGround())
        mBody->applyCentralImpulse(btVector3(0, impulse, 0));
}

bool Player::onGround() const
{
    btTransform trans;
    mBody->getMotionState()->getWorldTransform(trans);
    btVector3 vel = mBody->getLinearVelocity();
    return trans.getOrigin().getY() <= 1.05f && std::abs(vel.getY()) < 1.0f;
}

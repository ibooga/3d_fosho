#ifndef PLAYER_HPP
#define PLAYER_HPP

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <btBulletDynamicsCommon.h>

class Player
{
public:
    Player(Ogre::SceneManager* sceneMgr,
           btDiscreteDynamicsWorld* world,
           btAlignedObjectArray<btCollisionShape*>& collisionShapes);
    ~Player();

    void update();
    void applyMovement(const Ogre::Vector3& dir, float force);
    void jump(float impulse);
    bool onGround() const;

    Ogre::SceneNode* getNode() const { return mNode; }
    btRigidBody* getBody() const { return mBody; }

private:
    Ogre::SceneNode* mNode;
    btRigidBody* mBody;
    btDiscreteDynamicsWorld* mWorld;
};

#endif // PLAYER_HPP

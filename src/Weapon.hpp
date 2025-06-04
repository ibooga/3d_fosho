#ifndef WEAPON_HPP
#define WEAPON_HPP

#include <string>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

class GameApp;

class Weapon
{
public:
    Weapon(GameApp* app, const std::string& name, float rateOfFire, int ammo);
    virtual ~Weapon() = default;

    virtual void fire(const Ogre::Vector3& position, const Ogre::Quaternion& orient);
    void update(float dt);
    bool canFire() const { return mCooldown <= 0.0f && mAmmo != 0; }

    float getRateOfFire() const { return mRateOfFire; }
    int getAmmo() const { return mAmmo; }
    const std::string& getName() const { return mName; }

protected:
    GameApp* mApp;
    std::string mName;
    float mRateOfFire;
    int mAmmo;
    float mCooldown{0.0f};
};

class Pistol : public Weapon
{
public:
    Pistol(GameApp* app);
};

class Rifle : public Weapon
{
public:
    Rifle(GameApp* app);
};

#endif // WEAPON_HPP

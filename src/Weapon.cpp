#include "Weapon.hpp"
#include "GameApp.hpp"

Weapon::Weapon(GameApp* app, const std::string& name, float rateOfFire, int ammo)
    : mApp(app), mName(name), mRateOfFire(rateOfFire), mAmmo(ammo)
{
}

void Weapon::fire(const Ogre::Vector3& position, const Ogre::Quaternion& orient)
{
    if (!canFire())
        return;
    --mAmmo;
    mCooldown = 1.0f / mRateOfFire;
    if (mApp)
        mApp->createBullet(position, orient);
}

void Weapon::update(float dt)
{
    if (mCooldown > 0.0f)
        mCooldown -= dt;
}

Pistol::Pistol(GameApp* app)
    : Weapon(app, "Pistol", 2.0f, 12)
{
}

Rifle::Rifle(GameApp* app)
    : Weapon(app, "Rifle", 5.0f, 30)
{
}

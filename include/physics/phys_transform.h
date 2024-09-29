#ifndef __SUPSY_PHYS_TRANSFORM
#define __SUPSY_PHYS_TRANSFORM

#include <SupSy/SGE.h>

#define EXT_ID_PHYS_TRSFRM 150

typedef struct PhysicsTransform {
    sc_obj* source; // The scene object to which this is attached

    vec3 linVel;    // The linear velocity of the object (used mainly for initialization)
    vec3 angVel;    // The angular velocity of the object (used mainly for initialization)

    float mass;     // Mass of the solid
    mat3x3 inertia; // Moment of inertia

    uint index;     // Index in physics objects array
} phys_trsfrm;

/// @brief Free a physics transform
/// @param toFree The physics transform to free
void freePhysicsTransform(phys_trsfrm* toFree);

/// @brief Create a physics transform external data block
/// @param source The scene object to which this data is to be added
/// @param initialLinearVelocity The initial linear velocity of this object when starting the simulation
/// @param initialAngularvelocity The initial angular velocity of this object when starting the simulation
/// @param mass The mass of the object, should be stricly positive
/// @param momentOfInertia The moment of inertia for this object, should be non null
void addPhysicsTransformExtData(sc_obj* source, vec3 initialLinearVelocity, vec3 initialAngularvelocity, float mass, mat3x3 momentOfInertia);

/// @brief Calculate the inertia tensor for a sphere
/// @param mass The total mass
/// @param radius The radius
/// @return The resulting inertia tensor
mat3x3 physicsSphereInertiaTensorM(float mass, float radius);
/// @brief Calculate the inertia tensor for a sphere
/// @param density The average density
/// @param radius The radius
/// @return The resulting inertia tensor
mat3x3 physicsSphereInertiaTensorD(float density, float radius);

/// @brief Calculate the inertia tensor for a block
/// @param mass The total mass
/// @param X The length
/// @param Y The width
/// @param Z The height
/// @return The resulting inertia tensor
mat3x3 physicsBlockInertiaTensorM(float mass, float X, float Y, float Z);
/// @brief Calculate the inertia tensor for a block
/// @param density The average density
/// @param X The length
/// @param Y The width
/// @param Z The height
/// @return The resulting inertia tensor
mat3x3 physicsBlockInertiaTensorD(float density, float X, float Y, float Z);
/// @brief Calculate the inertia tensor for a cube
/// @param mass The total mass
/// @param side The side length
/// @return The resulting inertia tensor
mat3x3 physicsCubeInertiaTensorM(float mass, float side);
/// @brief Calculate the inertia tensor for a cube
/// @param density The average density
/// @param side The side length
/// @return The resulting inertia tensor
mat3x3 physicsCubeInertiaTensorD(float density, float side);

/// @brief Calculate the inertia tensor for a cylinder (Y axis)
/// @param mass The total mass
/// @param radius The radius
/// @param height The height
/// @return The resulting inertia tensor
mat3x3 physicsCylinderInertiaTensorM(float mass, float radius, float height);
/// @brief Calculate the inertia tensor for a cylinder (Y axis)
/// @param density The average density
/// @param radius The radius
/// @param height The height
/// @return The resulting inertia tensor
mat3x3 physicsCylinderInertiaTensorD(float density, float radius, float height);

#endif
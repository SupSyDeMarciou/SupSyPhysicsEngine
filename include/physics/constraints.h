#ifndef __SUPSY_PHYS_CONSTRAINTS
#define __SUPSY_PHYS_CONSTRAINTS

#include <SupSy/SGE.h>

#include "physics.h"
#include "phys_transform.h"

// Constraint function type
typedef void(func_constraint)(phys_trsfrm** constrainedObjects, void* additionalData, uint row, uint cidx, phys_syst* input);

// Constraint data type
typedef struct Constraint {

    phys_trsfrm** constrainedObjects;   // The objects interacting through this constraint
    void* additionalData;               // Any additionnal data needed when resolving the interaction
    func_constraint* constraintFunction; // The function calculating the result

    uint outputDegreesOfFreedom;        // The number of degrees of freedom the constraint removes from the system as a whole
    type ks, kd;                        // Compensating factors to account precision issues
} constraint;

/// @brief Free a constraint
/// @param toFree The constraint to free
void freeConstraint(constraint* toFree);

/// @brief Get the number of degree of freedom this contraint removes from the system
/// @param c The constraint
/// @return The number of degrees of freedom removed
uint constraintGetNbDegFreedom(constraint* c);

/// @brief Calculate and fill J, dJ, C and dC for this constraint
/// @param c The constraint
/// @param system Yhe current system
/// @param row The row of the constraint
/// @param idx The index of the constraint
void constraintCalculate(constraint* c, phys_syst* system, uint row, uint idx);

/*

CONSTRAINT CREATION GUIDE:

    1. Define an external data type for your constraint in which to hold any arbitray data you might need to calculate the constraint value
       Ex: vec3 pin for a fixed joint constraint
    
    2. Define a constraint function of type "func_constraint" (or which could be cast to said type) doing the following:
        Let C be the constraint function taking (p1,...,pn) arguments. It has to:
            a) Calculate dC/dpk for all k between 1 and n
            b) Calculate d(dC/dt)/dpk for all k between 1 and n
            c) Set J and dJ accordingly, with 0 in all places which dont correspond to a pk
    
    3. You may create a function creating a constraint to ease the process of reusability

*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// PRE-MADE CONSTRAINTS //////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Create a fixed joint constraint
/// @note Ensures the pin and a point on the object always overlapp
/// @param constrainedObject The object to be constrained
/// @param pin The pin in world-space
/// @return The newly created constraint
constraint* constraintNewFixedJoint(sc_obj* constrainedObject, vec3 pin);
/// @brief Create a joint constraint
/// @note Ensures two points in space always overlapp (point on object1 and point on object2)
/// @param constrainedObject1 The first object to be constrained
/// @param constrainedObject2 The second object to be constrained
/// @param pointOfContact The point of contatct between the two objects in world-space 
/// @return The newly created constraint
constraint* constraintNewJoint(sc_obj* constrainedObject1, sc_obj* constrainedObject2, vec3 pointOfContact);

/// @brief Create a fixed pivot constraint
/// @note Ensures the pin and a point on the object always overlapp and are oriented properly
/// @param constrainedObject The object to be constrained
/// @param pin The pin in world-space
/// @param axis The direction in world-space around which the object can rotate 
/// @return The newly created constraint
constraint* constraintNewFixedPivot(sc_obj* constrainedObject, vec3 pin, vec3 axis);
/// @brief Create a pivot constraint
/// @note Ensures two points in space always overlapp and are oriented properly (point on object1 and point on object2)
/// @param constrainedObject1 The first object to be constrained
/// @param constrainedObject2 The second object to be constrained
/// @param pin The pin in world-space
/// @param axis The direction in world-space around which the object can rotate 
/// @return The newly created constraint
constraint* constraintNewPivot(sc_obj* constrainedObject1, sc_obj* constrainedObject2, vec3 pointOfContact, vec3 axis);

/// @brief Create a fixed weld constraint
/// @note Ensures the pin and a point on the object always overlapp and maintain relative orientations
/// @param constrainedObject The object to be constrained
/// @param pin The pin in world-space
/// @return The newly created constraint
constraint* constraintNewFixedWeldJoint(sc_obj* constrainedObject, vec3 pin);
/// @brief Create a free weld constraint
/// @note Ensures two points in space always overlapp and maintain relative orientation (point on object1 and point on object2)
/// @param constrainedObject1 The first object to be constrained
/// @param constrainedObject2 The second object to be constrained
/// @param pin The pin in world-space
/// @return The newly created constraint
constraint* constraintNewWeldJoint(sc_obj* constrainedObject1, sc_obj* constrainedObject2, vec3 pointOfContact);

#endif
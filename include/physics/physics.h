#ifndef __SUPSY_PHYS_PHYSICS
#define __SUPSY_PHYS_PHYSICS

#include <stdarg.h>

#include "phys_transform.h"
#include "../../typeMaths/typeMaths.h"

#define NB_STATE_PARAMS 6
/// @brief A representation of a physical system
typedef struct PhysicsSystem
{
    // OBJECTS
    struct Constraint** constraints; // Constraints
    uint nbConstraints;             // Number of constraints
    phys_trsfrm** physicsObjects;   // Objects
    uint nbPhysObjects;             // Number of objects

    // STATE
    tlvec* q;                       // Global state vector
    tmat3x3* r;                     // Rotational matrices
    tlvec* dq;                      // Global velocity vector
    struct {
        tlvec* invM;                    // Inverse mass (float)
        tmat3x3* invIBase;              // Inverse moment of inertia at origin
        tmat3x3* invI;                  // Current inverse moment of inertia (rotated)
    } W;                            // The inverse inertia matrix
    tlvec* Fext;                    // External forces

    // CONSTRAINT
    tlvec* C;                       // The constraint vector
    tlvec* dC;                      // The derivative of the constraint vector relative to time
    block_tmat* J;                  // Jacobian matrix of constraints vector
    block_tmat* dJ;                 // Derivative Jacobian matrix of constraints vector relative to time

    // RESOLUTION
    block_tmat* leftMember;         // Left member of the equation
    block_tmat* P;                  // Conditionned left member
    tlvec* rightMember;             // Right member of the equation
    tlvec* X;                       // The solution to the linear equation
    block_tmat* JW;                 // J * M^-1
    tlvec* Fc;                      // The constraint force (what we are looking for)
    tlvec* tempC;                   // Just a vector to hold results of vector opperations

    // RK4
    tlvec* qInit;                   // The positionnal state of the system at the start of the RK4 call
    tlvec* dqInit;                  // The velocity state of the system at the start of the RK4 call
    tlvec* dqAcum;                  // The accumulated velocity over RK4
    tlvec* fAcum;                   // The accumulated force over RK4
    
    // OUTPUT
    tlvec* last_q;                  // Global state vector at t-dt
} phys_syst;

/// @brief Create a physics system in which to resolve constraints
phys_syst* newPhysicsSystem();
/// @brief Free the physics system
/// @param toFree The system to free
void freePhysicsSystem(phys_syst* toFree);

/// @brief Initialize the system with the simulated objects
/// @param nbPhysObjs The number of simulated objects
/// @param ... The scene objects to simulate
/// @warning Simulated objects must have a physics transform
void physicsSetSimulatedObjects(phys_syst* system, uint nbPhysObjs, ...);
/// @brief Initialize the system with the simulated objects
/// @param nbPhysObjs The number of simulated objects
/// @param objects The scene objects to simulate
/// @warning Simulated objects must have a physics transform
void physicsSetSimulateObjects_Array(phys_syst* system, uint nbPhysObjs, sc_obj** objects);

/// @brief Initialize the system with the applied constraints
/// @param nbConstraints The number of constraints
/// @param ... The constraints to apply
void physicsSetConstraints(phys_syst* system, uint nbConstraints, ...);
/// @brief Initialize the system with the applied constraints
/// @param nbConstraints The number of constraints
/// @param constraints The constraints to apply
void physicsSetConstraints_Array(phys_syst* system, uint nbConstraints, struct Constraint** constraints);

/// @brief Initialise the simulation
/// @param system The system to initialize
void physicsInitialize(phys_syst* system);

/// @brief Update positions of simulated objects
/// @param system The system from which to retrieve the positions
/// @param smooth Interpolation between current state and simulated state
void physicsGetUpdatedPositions(phys_syst* system, bool smooth);

/// @brief Start the physics simulation
/// @param system The physics system which will be simulated
/// @param nbIter The number of iterations per second
/// @param nbSubSteps The number of substeps to take at once
void physicsStartSimulation(phys_syst* system, uint nbIter, uint nbSubSteps, double factor);
/// @brief Set the current physics simulation state
/// @param active If the simulation is to be active
void physicsSetSimulationState(uint active);
/// @brief End the physics simulation
/// @param system The system currently simulated
void physicsEndSimulation(phys_syst* system);

#endif
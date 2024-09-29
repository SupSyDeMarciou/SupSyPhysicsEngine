# SupSyPhysicsEngine
## Introduction
A rigid body physics engine initialy made for the TIPE (one of the French engineering school entrance exams).

## Prerequisites
The SPE is made using the SGE and thus currently depends on it.
Instructions on how to download them are found on the [**SupSyGameEngine GitHub page**](https://github.com/SupSyDeMarciou/SupSyGameEngine).

The general syntax ruleset used throughout the project is identical to that of the SL and is outlined in the README.md of the [**SupSyLibraries GitHub page**](https://github.com/SupSyDeMarciou/SupSyLibraries).

## Content Overview
### Physics Transform
A **physics transform** is a **external data** block holding the physical properties of an object which makes it eligible for taking part in the simulation. Some basic *inertia tensors* can be calculated with functions prefixed with `physics[Shape]InertiaTensor` where `Shape` is either `Sphere`, `Box`, `Cube` or `Cylinder`. 

### Constraints
The SPE works with **constraints** to describre the expected evolution of the system. Contraints have their own *methods* under the prefix `constraint` and *constructors* under `constraintNew`.

### Physics System
The **physics system** holds all of the necessary information for resolving the constraints and step the simulation. It lives in the main thread but acts on a different thread to make the *simulation refresh rate* independent from the *frame rate*.

## Creating a simulation using the SPE
To make a **scene object** eligible for taking part in the simulation, you must attach a **physics transform** with `void addPhysicsTransformExtData(...);`.

To create a **constraint**, use one of the functions prefixed with `constraintNew`. If you want to define your own contraint to use inside of the simulation, refer to the comments in `SPE/physics/constraints.h`.

Before adding anything to the simulation, though, you must create a **physics system** in which the physics objects and constraints will be stored using `phys_syst* physicsSystem = newPhysicsSystem();`. Now that the system has been created, you can add all of the *physics objects* using `void physicsSetSimulatedObjects(physicsSystem, ...)` as well as all of the *constraints* with `void physicsSetConstraints(physicsSystem, ...)`. <br>
Finally, you can lock the system to initialize the simulation using `physicsInitialize(system);`. <br>
You can't add to or remove from this simulation anymore after calling this function.

Now we are ready to start simulating some rigid body physics! <br>
Using `void physicsStartSimulation(phys_syst* system, uint nbIter, uint nbSubSteps, double factor);`, the simulation has finally begun and our rigid bodies have started interacting with each others.
The simulation runs on a fixed update rate of `nbIter` updates per second, going `nbSubSteps` at a time and simulating `factor` seconds each *real-time* second. These parameters caracterize the discrete *time-step* $\Delta t$ used when calculating the evolution of the system as a whole.

Even though the simulation is currently running, nothing is moving on screen: we need to retrieve the new positions of our rigid bodies. This can be achieved by calling `physicsGetUpdatedPositions(PHYSICS_SYSTEM, false);` inside of the main *update* loop.
In addition, if, at any point, we want to pause the simulation, we can change its *running state* using `void physicsSetSimulationState(bool active);`. Furthermore, parenting a **trail** scene object to some of the rigid bodies might help visualize their path better (this is the reason why this object was created in the first place). It can be found at `SGE/builtin/objects/trail.h`.

Finally, the simulation can be stopped using `void physicsEndSimulation(phys_syst* system);`.
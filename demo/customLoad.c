#include <SGE.h>

#include <SPE.h>

static shader surfaceShader;
static material* armMat;
static material* pivotMat;
static material* massMat;
static mesh* cubeMesh;
static mesh* sphereMesh;
static mesh* cylinderMesh;
void initializeLoad(const char* surfaceMatPath) {
    surfaceShader = createSurfaceShader(surfaceMatPath);
    armMat = newStandardMaterial(surfaceShader, Vec4(0.1, 0.1, 0.15, 1), 0.1, 1.0, 1.0, vec3_zero);
    pivotMat = newStandardMaterial(surfaceShader, Vec4(0.02, 0.02, 0.03, 1), 0.1, 0.0, 1.0, vec3_zero);
    massMat = newStandardMaterial(surfaceShader, Vec4(0.02, 0.02, 0.03, 1), 0.01, 0.0, 1.0, vec3_zero);
    cubeMesh = meshUnitCube();
    sphereMesh = meshUnitUVShpere(64, 128);
    cylinderMesh = meshCylinder(64, 0.5, 0.5);
}

#include <SupSy/SGE/builtin/extData/freeCam.h>
#include <SupSy/SGE/builtin/extData/mouseCam.h>
#include <SupSy/SGE/builtin/objects/trail.h>
void createDefaultScene() {

    material* floorMat = newStandardMaterial(surfaceShader, Vec4(0.1, 0.1, 0.1, 1), 0.8, 0.0, 1.0, Vec3(0, 0, 0));
    sc_obj* plane = newSceneObject(newVec3(0, 0, 0), identityQ_(), vec3One_(10), NULL, true, true, NULL);
    scobjAddRenderObjectExtData_SingleMat(plane, meshUnitQuad(), floorMat, 1, RENDER_CULL_FRONT, 1, NULL, NULL);

    material* sphereMat = newStandardMaterial(surfaceShader, Vec4(1.0, 0.5, 0.1, 1), 0.5, 0.0, 1.0, Vec3(0, 0, 0));
    sc_obj* sphere = newSceneObject(newVec3(2, 0.5, 2), identityQ_(), vec3One_(1.0), NULL, true, true, NULL);
    scobjAddRenderObjectExtData_SingleMat(sphere, meshUnitUVShpere(128, 256), sphereMat, 1, RENDER_CULL_FRONT, 1, NULL, NULL);

    sc_obj* sun = newSceneObject(null3_(), newQuat_Euler(70, 60, 0), vec3One_(1), NULL, true, false, NULL);
    scobjAddLighExtData_Directional(sun, scale3(Vec3(1.0, 1.0, 0.95), 15.0));
    REbackground_skySetSun(scobjGetExtData(sun, EXT_ID_LIGHT));

    sc_obj* cam = freeCam_addDefault(newVec3(0, 1.0, -2), identityQ_(), 60, true);

    RESetRenderCamera(APP->renderEnvironment, scobjGetExtData(cam, EXT_ID_CAMERA));
}

static sc_obj* createArm(vec3 pin1, vec3 pin2, float width, float density) {
    vec3 mid = scale3(add3(pin1, pin2), 0.5);
    vec3 dir = sub3(pin1, pin2);
    float length = len3(dir);
    dir = norm3(dir);

    sc_obj* arm = newSceneObject(copy3_(&mid, NULL), newQuat_FromToVec(&vec3_forward, &dir), newVec3(width, width, length), NULL, true, false, NULL);
    scobjAddRenderObjectExtData_SingleMat(arm, cubeMesh, armMat, 1, RENDER_CULL_FRONT, 1, NULL, NULL);
    if (density > 0.0) {
        float mass = width * width * length * density;
        addPhysicsTransformExtData(arm, vec3_zero, vec3_zero, mass, physicsBlockInertiaTensorM(mass, width, width, length));
    }
    
    return arm;
}

static sc_obj* createBall(material* mat, vec3 pin, float diam, float mass) {

    sc_obj* ball = newSceneObject(copy3_(&pin, NULL), identityQ_(), vec3One_(diam), NULL, true, false, NULL);
    scobjAddRenderObjectExtData_SingleMat(ball, sphereMesh, mat, 1, RENDER_CULL_FRONT, 1, NULL, NULL);
    if (mass > 0.0) addPhysicsTransformExtData(ball, vec3_zero, vec3_zero, mass, physicsSphereInertiaTensorM(mass, diam * 0.5));

    return ball;
}

static sc_obj* createMass(vec3 pin, float diam, float mass) {
    return createBall(massMat, pin, diam, mass);
}

static sc_obj* createRotule(vec3 pin, float diam) {
    return createBall(pivotMat, pin, diam, -1.0);
}

static sc_obj* createPivot(vec3 pin, vec3 axis, float diam, float mass) {

    sc_obj* pivot = newSceneObject(copy3_(&pin, NULL), newQuat_FromToVec(&vec3_up, &axis), vec3One_(diam), NULL, true, false, NULL);
    scobjAddRenderObjectExtData_SingleMat(pivot, cylinderMesh, pivotMat, 1, RENDER_CULL_FRONT, 1, NULL, NULL);
    if (mass > 0.0) addPhysicsTransformExtData(pivot, vec3_zero, vec3_zero, mass, physicsCylinderInertiaTensorM(mass, diam*0.5, diam*0.5));

    return pivot;
}
static sc_obj* createPivot_Lit(vec3 pin, vec3 axis, float diam, float mass, vec3 color) {

    sc_obj* pivot = newSceneObject(copy3_(&pin, NULL), newQuat_FromToVec(&vec3_up, &axis), vec3One_(diam), NULL, true, false, NULL);
    material* mat = newStandardMaterial(surfaceShader, vec4_zero, 0.0, 0.0, 0.0, color);
    scobjAddRenderObjectExtData_SingleMat(pivot, cylinderMesh, mat, 1, RENDER_CULL_FRONT, 1, NULL, NULL);
    if (mass > 0.0) addPhysicsTransformExtData(pivot, vec3_zero, vec3_zero, mass, physicsCylinderInertiaTensorM(mass, diam*0.5, diam*0.5));

    return pivot;
}

phys_syst* createRope(int simulate, phys_syst* system, float weight, float angle, int segments, float length) {

    // Create objects
    vec3 pins[64 + 1] = {0}; // +1 for last bar
    sc_obj* physObjs[64 + 1] = {NULL}; // +1 for weight
    uint nbPhysObj = segments;
    constraint* constraints[64 + 1] = {NULL}; // +1 for weight
    uint nbConstraints = segments;
    pins[0] = Vec3(0, 0.5 + length, 0);
    sc_obj* pivot = createRotule(pins[0], 0.15);

    float dl = length / segments;
    float da = angle / segments;
    float angleC = 0.0;
    for (uint i = 1; i <= segments; i++) {
        angleC += da;
        pins[i] = add3(pins[i-1], Vec3(cos(angleC * DEG_TO_RAD)*dl, 0.0, sin(angleC * DEG_TO_RAD)*dl));
        physObjs[i-1] = createArm(pins[i-1], pins[i], 0.1, 1000.0);
    }

    // Create physics
    if (!simulate) return system;
    if (!system) failWithError("system not initialized", 0);

    constraints[0] = constraintNewFixedJoint(physObjs[0], (pins[0]));
    for (uint i = 1; i < segments; i++) {
        constraints[i] = constraintNewJoint(physObjs[i-1], physObjs[i], (pins[i]));
    }

    if (weight > 0.0) {
        physObjs[segments] = createMass(pins[segments], 0.3, weight);
        scobjAddLighExtData_Point(physObjs[segments], Vec3(15, 0, 0), 1.0, 1.0);
        newTrail(physObjs[segments], 0.1, 0.01, Vec3(1.0, 0.0, 0.0), vec3_one);
        constraints[segments] = constraintNewJoint(physObjs[segments-1], physObjs[segments], (pins[segments]));
        nbPhysObj++;
        nbConstraints++;
    }
    else {
        physObjs[segments - 1] = createMass(pins[segments - 1], 0.3, weight);
        scobjAddLighExtData_Point(physObjs[segments - 1], Vec3(15, 0, 0), 1.0, 1.0);
    }

    physicsSetSimulateObjects_Array(system, nbPhysObj, physObjs);
    physicsSetConstraints_Array(system, nbConstraints, constraints);
    physicsInitialize(system);
    
    return system;
}

phys_syst* createDoublePendulum(int simulate, phys_syst* system, float scale, float angle, float weight) {

    float armLength = 1 * scale;
    float rad = 0.25 * scale;
    float width = 0.1 * scale, density = 1000.0;
    vec3 axis = Vec3(cos(angle * DEG_TO_RAD), sin(angle * DEG_TO_RAD), 0);
    
    // Create objects
    vec3 pin1 = Vec3(0, 3.5, 0);
    sc_obj* pivot1 = createPivot(pin1, axis, rad, -1.0);

    vec3 pin2 = add3(pin1, Vec3(0, 0, armLength));
    sc_obj* arm1 = createArm(pin1, pin2, width, density);

    sc_obj* pivot2 = createPivot(pin2, axis, rad, weight > 0 ? weight : 1.0);
    scobjAddLighExtData_Point(pivot2, Vec3(0, 15, 0), 1.0, scale);
    vec3 trailCol2 = Vec3(2.5, 10.0, 2.5);
    newTrail(pivot2, 0.1 * scale, 0.01, trailCol2, trailCol2);

    vec3 pin3 = add3(pin2, Vec3(0, armLength, 0));
    sc_obj* arm2 = createArm(pin2, pin3, width, density);

    sc_obj* pivot3 = createPivot(pin3, axis, rad, weight > 0 ? weight : 1.0);
    scobjAddLighExtData_Point(pivot3, Vec3(15, 0, 0), 1.0, scale);
    vec3 trailCol3 = Vec3(10.0, 2.5, 2.5);
    newTrail(pivot3, 0.1 * scale, 0.01, trailCol3, trailCol3);

    // Create physics
    if (!simulate) return system;
    if (!system) failWithError("system not initialized", 0);

    physicsSetSimulatedObjects(system, 4, arm1, pivot2, arm2, pivot3);
    physicsSetConstraints(system, 4,
        constraintNewFixedPivot(arm1, pin1, axis),
        constraintNewPivot(arm1, pivot2, pin2, axis),
        constraintNewPivot(pivot2, arm2, pin2, axis),
        constraintNewPivot(arm2, pivot3, pin3, axis)
    );
    physicsInitialize(system);
}

phys_syst* createTriplePendulum(int simulate, phys_syst* system, float scale, float angle, float weight) {

    float armLength = 1 * scale;
    float rad = 0.25 * scale;
    float width = 0.1 * scale, density = 1000.0;
    vec3 axis = Vec3(cos(angle * DEG_TO_RAD), sin(angle * DEG_TO_RAD), 0);
    
    // Create objects
    vec3 pin1 = Vec3(0, 3.5, 0);
    sc_obj* pivot1 = createPivot(pin1, axis, rad, -1.0);

    vec3 pin2 = add3(pin1, Vec3(0, 0, armLength));
    sc_obj* arm1 = createArm(pin1, pin2, width, density);

    sc_obj* pivot2 = createPivot(pin2, axis, rad, weight > 0 ? weight : 1.0);
    scobjAddLighExtData_Point(pivot2, Vec3(15, 0, 0), 1.0, scale);
    vec3 trailCol2 = Vec3(10.0, 2.5, 2.5);
    newTrail(pivot2, 0.1 * scale, 0.01, trailCol2, trailCol2);

    vec3 pin3 = add3(pin2, Vec3(0, 0, armLength));
    sc_obj* arm2 = createArm(pin2, pin3, width, density);

    sc_obj* pivot3 = createPivot(pin3, axis, rad, weight > 0 ? weight : 1.0);
    scobjAddLighExtData_Point(pivot3, Vec3(0, 15, 0), 1.0, scale);
    vec3 trailCol3 = Vec3(2.5, 10.0, 2.5);
    newTrail(pivot3, 0.1 * scale, 0.01, trailCol3, trailCol3);

    vec3 pin4 = add3(pin3, Vec3(0, 0, armLength));
    sc_obj* arm3 = createArm(pin3, pin4, width, density);

    sc_obj* pivot4 = createPivot(pin4, axis, rad, weight > 0 ? weight : 1.0);
    scobjAddLighExtData_Point(pivot4, Vec3(0, 0, 15), 1.0, scale);
    vec3 trailCol4 = Vec3(2.5, 2.5, 10.0);
    newTrail(pivot4, 0.1 * scale, 0.01, trailCol4, trailCol4);

    // Create physics
    if (!simulate) return system;
    if (!system) failWithError("system not initialized", 0);

    physicsSetSimulatedObjects(system, 6, arm1, pivot2, arm2, pivot3, arm3, pivot4);
    physicsSetConstraints(system, 6,
        constraintNewFixedPivot(arm1, pin1, axis),
        constraintNewPivot(arm1, pivot2, pin2, axis),
        constraintNewPivot(pivot2, arm2, pin2, axis),
        constraintNewPivot(arm2, pivot3, pin3, axis),
        constraintNewPivot(pivot3, arm3, pin3, axis),
        constraintNewPivot(arm3, pivot4, pin4, axis)
    );
    physicsInitialize(system);
}

static sc_obj* createWheel(vec3 pin, vec3 axis, float diam, float mass) {

    sc_obj* wheel = newSceneObject(copy3_(&pin, NULL), newQuat_FromToVec(&vec3_forward, &axis), vec3One_(diam), NULL, true, false, NULL);

    scobjAddRenderObjectExtData_SingleMat(wheel, meshTransform(meshClone(cylinderMesh), scale3_(&axis, 0.2, NULL), identityQ_(), newVec3(1.0, 1.0, 0.2), 1), pivotMat, 1, RENDER_CULL_FRONT, 1, NULL, NULL);
    if (mass > 0.0) addPhysicsTransformExtData(wheel, vec3_zero, vec3_zero, mass, physicsCylinderInertiaTensorM(mass, diam*0.5, diam*0.5*0.2));

    return wheel;
}

phys_syst* createTrebuchet(int simulate, phys_syst* system, float angle, float weight) {

    vec3 pin_w_ul = Vec3(-1, 0.15, 0.5);
    vec3 pin_w_ur = Vec3(1, 0.15, 0.5);
    vec3 pin_w_dl = Vec3(-1, 0.15, -0.5);
    vec3 pin_w_dr = Vec3(1, 0.15, -0.5);

    vec3 pin_t_u = Vec3(0, 1.5, 0.3);
    vec3 pin_t_d = Vec3(0, 1.5, -0.3);

    float width = 0.1, density = 1000.0;
    
    // BODY
    // base rect
    sc_obj* arm1 = createArm(pin_w_ul, pin_w_ur, width, density);
    sc_obj* arm2 = createArm(pin_w_dl, pin_w_dr, width, density);
    sc_obj* arm3 = createArm(pin_w_ul, pin_w_dl, width, density);
    sc_obj* arm4 = createArm(pin_w_ur, pin_w_dr, width, density);

    // base cross
    sc_obj* arm5 = createArm(pin_w_ur, pin_w_dl, width, density);
    sc_obj* arm6 = createArm(pin_w_ul, pin_w_dr, width, density);

    // front support
    sc_obj* arm7 = createArm(pin_w_ul, pin_t_u, width, density);
    sc_obj* arm8 = createArm(pin_w_ur, pin_t_u, width, density);
    // Back support
    sc_obj* arm9 = createArm(pin_w_dl, pin_t_d, width, density);
    sc_obj* arm10 = createArm(pin_w_dr, pin_t_d, width, density);

    // pivot
    sc_obj* arm11 = createArm(pin_t_u, pin_t_d, width, density);

    // diagonal supports
    sc_obj* arm12 = createArm(pin_w_dl, pin_t_u, width, density);
    sc_obj* arm13 = createArm(pin_w_ur, pin_t_d, width, density);

    float wheelRad = 0.3, wheelMass = 100.0;
    sc_obj* wheel1 = createWheel(pin_w_dl, vec3_back, wheelRad, wheelMass);
    sc_obj* wheel2 = createWheel(pin_w_dr, vec3_back, wheelRad, wheelMass);
    sc_obj* wheel3 = createWheel(pin_w_ul, vec3_forward, wheelRad, wheelMass);
    sc_obj* wheel4 = createWheel(pin_w_ur, vec3_forward, wheelRad, wheelMass);

    // ARM
    vec3 pin_t_m = scale3(add3(pin_t_u, pin_t_d), 0.5);
    vec3 armDir = Vec3(1, 1, 0);
    vec3 armAxis = Vec3(0, 0, 1);
    vec3 pin_a_1 = addS3(pin_t_m, armDir, 1);
    vec3 pin_a_2 = addS3(pin_t_m, armDir, -1);
    sc_obj* arm14 = createArm(pin_a_1, pin_a_2, width, density);

    vec3 pin_a_3 = addS3(pin_a_1, vec3_down, 0.5);
    sc_obj* arm15 = createArm(pin_a_1, pin_a_3, width, density);

    sc_obj* mass = createMass(pin_a_3, 0.3, weight);

    if (!simulate) return system;
    if (!system) failWithError("system not initialized", 0);

    physicsSetSimulatedObjects(system, 20, arm1, arm2, arm3, arm4, arm5, arm6, arm7, arm8, arm9, arm10, arm11, arm12, arm13, arm14, arm15, mass, wheel1, wheel2, wheel3, wheel4);
    physicsSetConstraints(system, 31,
        // constraintNewFixedJoint(wheel1, pin_w_dl),
        // constraintNewFixedJoint(wheel2, pin_w_dr),
        // constraintNewFixedJoint(wheel3, pin_w_ul),
        // constraintNewFixedJoint(wheel4, pin_w_ur),

        // constraintNewPivot(arm1, wheel4, pin_w_ur, vec3_forward),
        constraintNewJoint(arm1, wheel4, pin_w_ur),
        // constraintNewPivot(arm2, wheel1, pin_w_dl, vec3_forward),
        constraintNewJoint(arm2, wheel1, pin_w_dl),
        // constraintNewPivot(arm3, wheel3, pin_w_ul, vec3_forward),
        constraintNewJoint(arm3, wheel3, pin_w_ul),
        // constraintNewPivot(arm4, wheel2, pin_w_dr, vec3_forward),
        constraintNewJoint(arm4, wheel2, pin_w_dr),

        constraintNewFixedJoint(arm1, (pin_w_ur)),
        constraintNewFixedJoint(arm2, (pin_w_dr)),
        constraintNewFixedJoint(arm3, (pin_w_ul)),
        constraintNewFixedJoint(arm4, (pin_w_dl)),

        constraintNewJoint(arm1, arm3, (pin_w_ul)),
        constraintNewJoint(arm1, arm4, (pin_w_ur)),
        constraintNewJoint(arm2, arm4, (pin_w_dr)),
        constraintNewJoint(arm2, arm3, (pin_w_dl)),

        // 12

        constraintNewJoint(arm5, arm4, (pin_w_ur)),
        constraintNewJoint(arm5, arm3, (pin_w_dl)),
        constraintNewJoint(arm6, arm2, (pin_w_dr)),
        constraintNewJoint(arm6, arm1, (pin_w_ul)),

        constraintNewJoint(arm7, arm6, (pin_w_ul)),
        constraintNewJoint(arm8, arm5, (pin_w_ur)),
        constraintNewJoint(arm9, arm5, (pin_w_dl)),
        constraintNewJoint(arm10, arm6, (pin_w_dr)),
        constraintNewJoint(arm7, arm8, (pin_t_u)),
        constraintNewJoint(arm9, arm10, (pin_t_d)),

        constraintNewJoint(arm11, arm7, (pin_t_u)),
        constraintNewJoint(arm11, arm9, (pin_t_d)),

        // 24

        constraintNewJoint(arm12, arm11, (pin_t_u)),
        constraintNewJoint(arm12, arm9, (pin_w_dl)),
        constraintNewJoint(arm13, arm11, (pin_t_d)),
        constraintNewJoint(arm13, arm8, (pin_w_ur)),

        // constraintNewPivot(arm14, arm11, pin_t_m, armAxis),
        constraintNewJoint(arm14, arm11, (pin_t_m)),
        constraintNewJoint(arm15, arm14, (pin_a_1)),
        constraintNewJoint(mass, arm15, (pin_a_3))
    );
    physicsInitialize(system);
}
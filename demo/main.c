#define SGE_BASE_WIDTH 1920*4
#define SGE_BASE_HEIGHT 1080*4

#include <SGE.h>
#include <SGE/builtin/postEffects/bloom.h>

#include "customLoad.c"

int main(int argc, const char* argv[]) {
    bool checks[3] = {false, false, true};
    uint PHYSICS_ITERATIONS = 10000;
    uint PHYSICS_SUBSTEPS = 50; // SUBSTEPS should be bellow ITERATIONS / 150
    uint PHYSICS_FACTOR = 1.0;
    for (int i = 0; i < argc; i++) {
        const char* c = argv[i];
        if (SL_cmpStr(c, "phys")) {
            checks[0] = true;
            sscanf(argv[++i], "%u", &PHYSICS_ITERATIONS);
            if (i+1 < argc && SL_cmpStr(argv[i+1], "false")) { checks[2] = false; i++; }
            else if (i+1 < argc && SL_cmpStr(argv[i+1], "true")) i++;
        }
        else if (SL_cmpStr(c, "substep")) {
            if (!checks[0]) {
                printf("Substeps require that \"phys [iteration] {start active = true}\" be initialized first");
                exit(1);
            }
            sscanf(argv[++i], "%u", &PHYSICS_SUBSTEPS);
        }
        else if (SL_cmpStr(c, "free")) checks[1] = true;
        else if (SL_cmpStr(c, "factor")) {
            if (!checks[0]) {
                printf("Factor require that \"phys [iteration] {start active = true}\" be initialized first");
                exit(1);
            }
            sscanf(argv[++i], "%f", &PHYSICS_FACTOR);
        }
    }

    printf("Simulation active: %s\n", checks[0] ? "true" : "false");
    if (checks[0]) {
        printf("\t# Iterations: %d\n\t# Substeps: %d\n", PHYSICS_ITERATIONS, PHYSICS_SUBSTEPS);
    }

    initializeApp("Phys v2.4");
    initializeLoad("!builtin/shaders/pbr.fs");
    initializeBloomLogic();

    ///// SCENE DATA
    phys_syst* PHYSICS_SYSTEM = newPhysicsSystem();
    createDefaultScene();
    // createRope(checks[0], PHYSICS_SYSTEM, 10.0, 90.0, 20, 5);
    createDoublePendulum(checks[0], PHYSICS_SYSTEM, 1.0, 0.0, 20000.0);
    // createTriplePendulum(checks[0], PHYSICS_SYSTEM, 0.5, 30.0, 20000.0);
    // createTrebuchet(checks[0], PHYSICS_SYSTEM, 0.0, 100.0);

    if (checks[0]) physicsStartSimulation(PHYSICS_SYSTEM, PHYSICS_ITERATIONS, PHYSICS_SUBSTEPS, PHYSICS_FACTOR);
    physicsSetSimulationState(checks[2]);
    printf("created scene\n");
    /////

    frame_buffer* fb0 = REGetOutputFB(APP->renderEnvironment);
    frame_buffer* fb1 = newFrameBuffer(Uvec2(SGE_BASE_WIDTH, SGE_BASE_HEIGHT));
    FBAttachColor(fb1, tex2DGenFormat(TEX_COL_RGBA, TEX_BIT_DEPTH_16, TEX_FORM_FLOAT));
    FBFailIfNotComplete(fb1);

    bool running = checks[2];
    if (!running) printf("### Paused ###");
    while (!appShouldClose()) {
        startFrameUpdate();

        if (checks[0]) {
            if (inputIsKeyPressed(SGE_KEY_P)) {
                running = !running;
                physicsSetSimulationState(running);
                trailSimulate(running);
                if (!running) printf("### Paused ###");
                else printf("\r              \r");
            }
            physicsGetUpdatedPositions(PHYSICS_SYSTEM, false);
        }
        sceneUpdate(APP->scene);

        RErenderScene();
        blitBloomFB(fb0, fb1, 5, 0.1);
        blitHdrToLdrFB(fb1, fb0);
        blitToScreenFB(fb0);

        endFrameUpdate();
    }

    physicsEndSimulation(PHYSICS_SYSTEM);
    glfwTerminate();
    printf("Terminated\n");
    return EXIT_SUCCESS;
}

// You can then run "./main phys 2500 substep 2"
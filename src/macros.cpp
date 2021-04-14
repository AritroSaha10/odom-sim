#include "macros.h"
#include "control.h"
#include <chrono>
#include <thread>

bool suspendDrive = false;

int callbackCount = 0;

// -------------------- These don't do anything in the simulator due to not having access to motor functions -------------------- //
// --------------------- See https://github.com/jfssvex/vex2020-21/tree/master/src for what these represent --------------------- //

void flipout() {
    printf("FLIPOUT\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    printf("FLIPOUT COMPLETE\n");
    // rollers.flipout();
}

void in() {
    printf("INTAKING\n");
    // rollers.intake();
    // intake.intake(127);
}

void stopRollers() {
    printf("ROLLERS STOPPED\n");
    // rollers.stop();
    // intake.stop();
}

void shootClean() {
    printf("CLEAN SHOT\n");
    // rollers.eject();
}

void shootStaggered() {
    printf("MULTIPLE SHOT\n");
    // rollers.shoot();
}

void shootStaggeredIntake() {
    printf("MULTIPLE SHOT WITH INTAKE\n");
    // rollers.shoot();
    // intake.intake(127);
}
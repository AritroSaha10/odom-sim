#include "macros.h"
#include "control.h"
#include <chrono>
#include <thread>
#include <cstdio>

const int initialShotDelay = 400;    // ms
const int subsequentShotDelay = 400; // ms

void ballDelay(uint8_t balls) {
    auto d = initialShotDelay + (subsequentShotDelay * (balls - 1));
    std::this_thread::sleep_for(std::chrono::milliseconds(d));
}

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

void shootClean(uint8_t balls) {
    printf("CLEAN SHOT\n");
    ballDelay(balls);
    // rollers.eject();
}

void shootStaggered(uint8_t balls) {
    printf("MULTIPLE SHOT\n");
    ballDelay(balls);
    // rollers.shoot();
}

void shootStaggeredIntake(uint8_t balls) {
    printf("MULTIPLE SHOT WITH INTAKE\n");
    ballDelay(balls);
    // rollers.shoot();
    // intake.intake(127);
}
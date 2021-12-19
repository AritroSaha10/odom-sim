#pragma once

#include "tracking.h"
#include <math.h>

#define SETTLE_DELAY 0.2

// Data structure for PID constants
typedef struct PIDInfo {
    double p, i, d, motor;

    PIDInfo(double _p, double _i, double _d);
    PIDInfo() {};
} PIDInfo;

// Manages closed PID loops
typedef struct PIDController {
private:
    // PID vars
	double sense;
	double lastError;
	double integral;
	double error, derivative, speed;
    double target;

    // settling/state
    double settleStart;
    bool settling, settled = false;
    bool first = true;

    // constants
    double tolerance;
    double integralTolerance;
    PIDInfo constants;

public:
	PIDController(double _target, PIDInfo _constants, double _tolerance, double _integralTolerance);

    double step(double newSense);
    void reset();

    double getError();
    bool isSettled();

} PIDController;

void strafe(Vector2 dir, double turn);
void strafeToOrientation(Vector2 target, double angle);
void strafeToPoint(Vector2 target);
void turnAndMoveToPoint(Vector2 target);
void turnAndMoveToPoint2(Vector2 target);
void turnToAngle(double target);
void turnRelative(double target);
void alignAndShoot(Vector2 goal, double angle, uint8_t balls, bool intake = false);

void strafeRelative(Vector2 offset, double aOffset = 0);
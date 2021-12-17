#pragma once
#include "Eigen/Dense"
#include <cmath>
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <vector>
#include <chrono>
#include "math.h"
#define _USE_MATH_DEFINES

// Robot metrics (in)
const double wheelbase = 14;
const double backOffset = 0;

// Manually tuned physical constants
const double angularStoppingDecel = 10 * M_PI;
const double stoppingDecel = 70;
const double acceleration = 100 + stoppingDecel;
const double angularAcceleration = 4 * M_PI;
const double maxSpeed = 48;
const double maxAngularSpeed = 4 * M_PI;

const double viscousFrictionCoeff = 0.5;
const double viscousAngularCoeff = 0.75;

const double maxForce = acceleration / 2.828;

// GL containers
extern GLuint driveVBO;
extern GLuint driveVAO;

// Starting configuration (in/rad)
#define STARTX 0
#define STARTY 0
#define STARTO 0

// angle conversion
double degToRad(double d);
double radToDeg(double r);

// Simulate a motor
class Motor {
private:
    // Constants
    const double jerk = 10000; // Consider very high (near instant) jerk
    glm::vec2 position;
    double orientation;

    // State
    double output;
    double lastUpdate = 0;

public:
    // Constructors
    Motor();
    Motor(glm::vec2 p, double o);

    void setPower(double power);
    glm::vec2 getForce();
    glm::vec2 getPosition();
};

// Class that manages the simulated holonomic chassis
class XDrive {
private:
    // State
    glm::vec2 localVelocity;
    double angularVelocity;

    glm::vec2 position;
    double orientation;
    double lastUpdate;

    // Drive motors in X-drive configuration
    std::vector<Motor> motors = {
        Motor(glm::vec2( wheelbase / 24,  wheelbase / 24), degToRad( 0)), // front right
        Motor(glm::vec2(-wheelbase / 24,  wheelbase / 24), degToRad(-0)), // front left
        Motor(glm::vec2( wheelbase / 24, -wheelbase / 24), degToRad(-0)), // back  right
        Motor(glm::vec2(-wheelbase / 24, -wheelbase / 24), degToRad( 0))  // back  left
    };

    // Forces
    glm::vec2 getNetForce();
    double getNetTorque();

public:
    // Constructors
    XDrive();
    XDrive(glm::vec2 startPos);
    XDrive(glm::vec2 startPos, double startOrientation);

    // Interface
    void strafe(glm::vec2 drive, double turn);
    void update();
    void strafeGlobal(glm::vec2 dir, double turn);

    // Utility
    glm::vec2 localToGlobal(glm::vec2 v);
    glm::vec2 globalToLocal(glm::vec2 vec);
    glm::mat4 getMatrix();

    // Getters
    glm::vec2 getPosition();
    double getOrientation();
};

glm::vec2 rotateVector(glm::vec2 v, double angle);
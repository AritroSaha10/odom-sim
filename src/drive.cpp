#include "odom.h"
#include "tracking.h"
#include "glm/gtc/matrix_transform.hpp"
#include <iostream>
#include <chrono>
#include <thread>

Vector2 glmToCustom(glm::vec2 v) {
    return Vector2(v.x, v.y);
}

glm::vec2 customToGLM(Vector2 v) {
    return glm::vec2(v.getX(), v.getY());
}

// constructors
XDrive::XDrive() : XDrive(glm::vec2(0), 0) { }

XDrive::XDrive(glm::vec2 startPos) : XDrive(startPos, 0) { }

XDrive::XDrive(glm::vec2 startPos, double startOrientation) {
    this->orientation = startOrientation;
    this->position = startPos;
}

/**
 * Drive the X-drive in a given direction, while also turning
 * Parameters are in global space
 * Meant to mimic the strafe function in the robot code
 * 
 * @param drive the direction in which to drive, ||dir|| <= 1
 * @param turn the speed to turn at, -1 <= turn <= 1
*/
void XDrive::strafeGlobal(glm::vec2 dir, double turn) {
    Vector2 cDir = toLocalCoordinates(glmToCustom(dir));
	double xVel = cDir.getX();
	double yVel = cDir.getY();

    strafe(glm::vec2(xVel, yVel), turn);
}

/**
 * Drive the X-drive in a given direction, while also turning
 * Parameters are in local space
 * 
 * @param drive the direction in which to drive, ||drive|| <= 1
 * @param turn the speed to turn at, -1 <= turn <= 1
*/
void XDrive::strafe(glm::vec2 drive, double turn) {
    double straight = drive.y;

    double leftOutput = straight + turn;
    double rightOutput = straight - turn;
    
    double scalar = std::max(std::abs(leftOutput), std::abs(rightOutput));
    if (scalar > 1) {
        leftOutput /= scalar;
        rightOutput /= scalar;
    }

    // printf("Left Motor Power: %f\n", (straight + turn) / scalar);
    // printf("Right Motor Power: %f\n", (straight - turn) / scalar);
    motors[0].setPower(rightOutput); // front right
    motors[1].setPower(leftOutput); // front left
    motors[2].setPower(rightOutput); // back  right
    motors[3].setPower(leftOutput); // back  left

    update();
}

void XDrive::tank(double leftOutput, double rightOutput) {
    double scalar = std::max(std::abs(leftOutput), std::abs(rightOutput));
    if (scalar > 1) {
        leftOutput /= scalar;
        rightOutput /= scalar;
    }

    // printf("Left Motor Power: %f\n", (straight + turn) / scalar);
    // printf("Right Motor Power: %f\n", (straight - turn) / scalar);
    motors[0].setPower(rightOutput); // front right
    motors[1].setPower(leftOutput); // front left
    motors[2].setPower(rightOutput); // back  right
    motors[3].setPower(leftOutput); // back  left

    update();
}

/**
 * Scale the magnitude of linear friction based on direction.
 * Because of X-drive configuration and omni-wheels, friction varies
 * depending on the direction of motion relative to the wheels.
 * Also applies viscous friction (things like drag and resistance
 * from lubricants, dependent on speed)
 * 
 * @param localVel the velocity of the robot in local space
 * @param friction the magnitude of sliding friction
*/
glm::vec2 calculateLinearFriction(glm::vec2 localVel, float friction) {
    glm::vec2 norm = glm::normalize(localVel);

    double straight = norm.y;
    double right = norm.x;

    double scalar = 1;
    if (abs(right) + abs(straight) > 1) {
        scalar = abs(right) + abs(straight);
    }

    Vector2 fr = rotateVector(Vector2(0, (straight - right) / scalar), degToRad( 0));  // front right
    Vector2 fl = rotateVector(Vector2(0, (straight + right) / scalar), degToRad(-0));  // front left
    Vector2 br = rotateVector(Vector2(0, (straight - right) / scalar), degToRad(-0));  // back  right
    Vector2 bl = rotateVector(Vector2(0, (straight + right) / scalar), degToRad( 0));  // back  left
    
    Vector2 net = fr + fl + br + bl;
    // std::cout << net.getMagnitude() << std::endl;

    glm::vec2 viscousFriction = -localVel * (float)viscousFrictionCoeff;

    return (float)(net.getMagnitude() * friction / 2) * -glm::vec2(norm.x, norm.y) + viscousFriction;
}

/**
 * Update the chassis' state depending on motor outputs
 * and past state. Also updates virtual tracking wheels
*/
void XDrive::update() {
    auto t = glfwGetTime();
    auto deltaT = t - lastUpdate;

    glm::vec2 driveForce = getNetForce();

    double angAccel = getNetTorque();
    glm::vec2 oldVelocity = localVelocity;
    localVelocity = localVelocity + (driveForce * (float)deltaT);
    if(glm::length(localVelocity) > maxSpeed) {
        localVelocity = glm::normalize(localVelocity) * (float)maxSpeed;
    }
    angularVelocity += angAccel * deltaT;
    if(abs(angularVelocity) > maxAngularSpeed) {
        angularVelocity = angularVelocity / abs(angularVelocity) * maxAngularSpeed;
    }

    // Angular resistance
    if(angularVelocity != 0) {
        double angVelSign = angularVelocity/abs(angularVelocity);
        double angFriction = (-angularStoppingDecel * deltaT * angVelSign)
            + (-angularVelocity * deltaT * viscousAngularCoeff);
        angularVelocity += angFriction;
        if(angVelSign != angularVelocity / abs(angularVelocity)) {
            angularVelocity = 0;
        }
    }

    // Linear resistance
    if(glm::length(localVelocity) != 0) {
        glm::vec2 lastVelDir = glm::normalize(localVelocity);
        glm::vec2 linearFriction = calculateLinearFriction(
            localVelocity * (float)deltaT,
            (float)(stoppingDecel * deltaT));
        localVelocity += linearFriction;
        if (glm::length(lastVelDir - glm::normalize(localVelocity)) > 0.1) {
            localVelocity = glm::vec2(0, 0);
        }
    }

    // Apply velocities to positions
    glm::vec2 velocity = localToGlobal(localVelocity);

    glm::vec2 oldPosition = position;
    position = position + ((float)deltaT * velocity);
    orientation += angularVelocity * deltaT;
    orientation = fmod(orientation, 2 * M_PI);
    lastUpdate = t;

    // Update tracking wheels
    Vector2 dP = glmToCustom(globalToLocal(position - oldPosition));
    double dO = angularVelocity * deltaT;

    leftTrackingWheel.update(dP, dO);
    rightTrackingWheel.update(dP, dO);
    backTrackingWheel.update(dP, dO);
}

/**
 * Returns the net force vector acting on the chassis
*/
glm::vec2 XDrive::getNetForce() {
    glm::vec2 net(0, 0);
    for(auto m : motors) {
        net += m.getForce();
    }
    return net;
}

/**
 * Returns the magnitude of net torque acting on the chassis
 * Positive = clockwise
*/
double XDrive::getNetTorque() {
    glm::vec3 net(0, 0, 0);
    for(auto m : motors) {
        net += glm::cross(glm::vec3(m.getPosition(), 0), glm::vec3(m.getForce(), 0));
    }
    return net.z;
}

/**
 * Rotate a glm direction vector to global space (relative to the field)
 * 
 * @param vec the vector to rotate
 * @return rotated vector
*/
glm::vec2 XDrive::localToGlobal(glm::vec2 vec) {
    return rotateVector(vec, orientation);
}

/**
 * Rotate a glm direction vector to local space (relative to the robot chassis)
 * 
 * @param vec the vector to rotate
 * @return rotated vector
*/
glm::vec2 XDrive::globalToLocal(glm::vec2 vec) {
    return rotateVector(vec, -orientation);
}

/**
 * Get the 4x4 matrix used to transform the chassis into the
 * correct position for rendering
 * 
 * @return the 4x4 matrix
*/
glm::mat4 XDrive::getMatrix() {
    glm::mat4 mat;
    mat = glm::scale(mat, glm::vec3(2.0f/144));
    mat = glm::translate(mat, glm::vec3(position, 0) + glm::vec3(0, backOffset + 2, 0) + glm::vec3(-144 / 2, -144 / 2, 0));
    mat = glm::rotate(mat, (float)orientation, glm::vec3(0, 0, 1));
    return mat;
}

/**
 * Rotate a 2 dimensional glm vector by a given angle
 * 
 * @param vec the vector
 * @param angle the angle in radians
*/
glm::vec2 rotateVector(glm::vec2 vec, double angle) {
    // x = cos(a), y = sin(a)
    // cos(a + b) = cos(a)cos(b) - sin(a)sin(b)
    double newX = (vec.x * cos(angle)) - (vec.y * sin(angle));

    // sin(a + b) = sin(a)cos(b) + cos(a)sin(b)
    double newY = (vec.y * cos(angle)) + (vec.x * sin(angle));

    return glm::vec2(newX, newY);
}

glm::vec2 XDrive::getPosition() {
    return this->position;
}

double XDrive::getOrientation() {
    return this->orientation;
}

// ----------------- Motor Class ----------------- //

Motor::Motor() {}

Motor::Motor(glm::vec2 p, double o) {
    this->position = p;
    this->orientation = o;
}

glm::vec2 Motor::getForce() {
    glm::vec2 f(0, output);
    return customToGLM(rotateVector(glmToCustom(f), orientation));
}

glm::vec2 Motor::getPosition() {
    return position;
}

/**
 * Command the motor to output at a certain power
 * 
 * @param power the desired output as a percentage, -1 <= power <= 1
*/
void Motor::setPower(double power) {
    if(abs(power) > 1) {
        power = power > 0 ? 1 : -1;
    }

    auto t = glfwGetTime();
    auto deltaT = t - lastUpdate;

    auto target = power * maxForce;

    // Check if power needs to go up or down
    auto delta = target - output;
    int deltaSign = delta/abs(delta);

    // Increment power
    output += deltaSign * jerk * deltaT;

    // If the output overshot then reset to the target
    delta = target - output;
    if(delta/abs(delta) != deltaSign) {
        output = target;
    }

    lastUpdate = t;
}
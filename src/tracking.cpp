#include "tracking.h"
#include "odom.h"
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

// Sensors
float rDelta, lDelta, bDelta;
std::vector<double> encoders;
// Temp
float rDist, lDist, aDelta, rLast, lLast, halfA;
float bDist, bLast;
// Constants and macros
const float lrOffset = WHEELBASE/2.0f; // 9.5 inch wheel base, halved
const float bOffset = -BACK_WHEEL_OFFSET;

// inches per encoder tick (degrees)
#define DRIVE_DEGREE_TO_INCH (M_PI * 3.25 / 360)
#define TRACKING_WHEEL_DEGREE_TO_INCH (M_PI * TRACKING_WHEEL_DIAMETER / 360)
#define TRACKING_WHEEL_INCH_TO_DEGREE (360 / (M_PI * TRACKING_WHEEL_DIAMETER))

// Define externs declared in header
TrackingData trackingData(STARTX, STARTY, STARTO);
VirtualEncoder leftTrackingWheel(-WHEELBASE / 2);
VirtualEncoder rightTrackingWheel(WHEELBASE / 2);
VirtualEncoder backTrackingWheel(BACK_WHEEL_OFFSET, true);

bool logPosition = false;

/**
 * Round a double up to n places
 * 
 * @param v the double to round
 * @param places the number of decimal places to keep
 * @return the rounded double
*/
double roundUp(double v, int places) {
    const double mult = std::pow(10.0, places);
	return std::ceil(v * mult) / mult;
}

/**
 * Convert an angle from radians to degrees
 * 
 * @param r the angle in radians
 * @return the angle in degrees
*/
double radToDeg(double r) {
	return r * 180 / M_PI;
}
/**
 * Convert and angle from degrees to radians
 * 
 * @param d the angle in degrees
 * @return the angle in radians
*/
double degToRad(double d) {
	return d * M_PI / 180;
}

/**
 * Track the chassis' pos/orientation given virtualized tracking
 * wheel data. Run in its own thread, thus while(1).
*/
void tracking() {

	// Initialize variables
	Vector2 globalPos{ trackingData.getX(), trackingData.getY() };

	float left = 0;
	float right = 0;
	float lateral = 0;
	float angle = 0;


	// Initialize variables
    lLast = 0; // Last encoder value of left
    rLast = 0; // Last encoder value of right
    bLast = 0; // Last encoder value of back

    // Tracking loop
    while (true) {
		Vector2 localPos{ 0, 0 };

        // Get encoder data, directly fron wheels because no tracking wheels yet
        float lEncVal = leftTrackingWheel.read();
        float rEncVal = rightTrackingWheel.read();
        float bEncVal = 0;

        // Calculate delta values
        lDelta = lEncVal - lLast;
        rDelta = rEncVal - rLast;
        bDelta = bEncVal - bLast;

        // Calculate IRL distances from deltas
        lDist = lDelta * DRIVE_DEGREE_TO_INCH;
        rDist = rDelta * DRIVE_DEGREE_TO_INCH;
        bDist = bDelta * DRIVE_DEGREE_TO_INCH;

        // Update last values for next iter since we don't need to use last values for this iteration
        lLast = lEncVal;
        rLast = rEncVal;
        bLast = bEncVal;

        // Update total distance vars
        left += lDist;
        right += rDist;
        lateral += bDist;

        // Calculate new absolute orientation
        float prevAngle = angle; // Previous angle, used for delta
        angle = (right - left) / WHEELBASE;

        // Get angle delta
        aDelta = angle - prevAngle;

        // Calculate using different formulas based on if orientation change
        float avgLRDelta = (lDist + rDist) / 2; // Average of delta distance travelled by left and right wheels
        if (aDelta == 0.0f) {
            // Set the local positions to the distances travelled since the angle didn't change
            localPos = Vector2(bDist, avgLRDelta);
        } else {
            // Use the angle to calculate the local position since angle did change
            localPos = Vector2(
                2 * sin(aDelta / 2) * (bDist / aDelta - bOffset),
                2 * sin(aDelta / 2) * (rDist / aDelta - lrOffset)
            );
        }

        // Calculate the average orientation
        // If any issues arise, try changing aDelta to aDelta/2
        float avgAngle = -(prevAngle + (aDelta / 2));

        // Calculate global offset https://www.mathsisfun.com/polar-cartesian-coordinates.html
        float globalOffsetX = cos(avgAngle); // cos(θ) = x 
        float globalOffsetY = sin(avgAngle); // sin(θ) = y 

        // Finally, update the global position
        globalPos = Vector2(
            trackingData.getPos().getX() + (localPos.getY() * globalOffsetY) + (localPos.getX() * globalOffsetX),
            trackingData.getPos().getY() + (localPos.getY() * globalOffsetX) - (localPos.getX() * globalOffsetY)
        );

        // Update tracking data
        // trackingData.update(globalPos.getX(), globalPos.getY(), degToRad(myImu.get_rotation()));
        trackingData.update(globalPos.getX(), globalPos.getY(), trackingData.getHeading() + aDelta);

        // Debug print
		printf("X: %f, Y: %f, A: %f\n", 
			trackingData.getPos().getX(), 
			trackingData.getPos().getY(), 
			-radToDeg(trackingData.getHeading())
		);

		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}

// ----------------- Math utilities ----------------- //

/**
 * Rotate a direction vector to local space (relative to the robot chassis)
 * 
 * @param vec the vector to rotate
 * @return rotated vector
*/
Vector2 toLocalCoordinates(Vector2 vec) {
	double localAngle = -trackingData.getHeading();

	return rotateVector(vec, localAngle);
}

/**
 * Rotate a direction vector to global space (relative to the field)
 * 
 * @param vec the vector to rotate
 * @return rotated vector
*/
Vector2 toGlobalCoordinates(Vector2 vec) {
	double localAngle = trackingData.getHeading();

	return rotateVector(vec, localAngle);
}

/**
 * Get the dot product of 2 vectors
 * 
 * @param v1 vector a
 * @param v2 vector b
 * @return a dot b
*/
double dot(Vector2 v1, Vector2 v2) {
	return (v1.getX() * v2.getX()) + (v1.getY() * v2.getY());
}

/**
 * Rotate a 2 dimensional vector by a given angle
 * 
 * @param vec the vector
 * @param angle the angle in radians
*/
Vector2 rotateVector(Vector2 vec, double angle) {
	// x = cos(a), y = sin(a)
	// cos(a + b) = cos(a)cos(b) - sin(a)sin(b)
	double newX = (vec.getX() * cos(angle)) - (vec.getY() * sin(angle));

	// sin(a + b) = sin(a)cos(b) + cos(a)sin(b)
	double newY = (vec.getY() * cos(angle)) + (vec.getX() * sin(angle));

	return Vector2(newX, newY);
}

// ----------------- Tracking Data Struct ----------------- //

/**
 * Create a TrackingData instance (one per chassis)
 * 
 * @param _x starting x coord in inches
 * @param _x starting y coord in inches
 * @param _h starting heading in radians
*/
TrackingData::TrackingData(double _x, double _y, double _h) {
	this->pos = Vector2(_x, _y);
	if(!suspendModulus) {
		this->heading = fmod(_h, 2 * M_PI);
	}
	else {
		this->heading = _h;
	}
}

double TrackingData::getX() {
	return pos.getX();
}
double TrackingData::getY() {
	return pos.getY();
}
double TrackingData::getHeading() {
	return heading;
}
Vector2 TrackingData::getPos() {
	return pos;
}
/**
 * Get the chassis' forward vector in global space
 * 
 * @return the forward vector (magnitude = 1)
*/
Vector2 TrackingData::getForward() {
	return toGlobalCoordinates(Vector2(0, 1));
}

/**
 * Update the chassis position
 * 
 * @param _x new x coord in inches
 * @param _x new y coord in inches
 * @param _h new heading in radians
*/
void TrackingData::update(double _x, double _y, double _h) {
	this->pos = Vector2(_x, _y);
	if(!suspendModulus) {
		this->heading = fmod(_h, 2 * M_PI);
	}
	else {
		this->heading = _h;
	}
}

void TrackingData::suspendAngleModulus() {
	suspendModulus = true;
}
void TrackingData::resumeAngleModulus() {
	suspendModulus = false;
}

// ----------------- Vector2 Struct ----------------- //

// Operator overrides for vector equivalents
Vector2 operator + (const Vector2 &v1, const Vector2 &v2) {
	Vector2 vec(v1.x + v2.x, v1.y + v2.y);
	return vec;
}
Vector2 operator - (const Vector2 &v1, const Vector2 &v2) {
	Vector2 vec(v1.x - v2.x, v1.y - v2.y);
	return vec;
}
Vector2 operator * (const Vector2 &v1, double scalar) {
	Vector2 vec(v1.x * scalar, v1.y * scalar);
	return vec;
}

Vector2::Vector2(double _x, double _y) {
	this->x = _x;
	this->y = _y;
}
Vector2::Vector2() {
	this->x = 0;
	this->y = 0;
}

double Vector2::getX() {
	return x;
}
double Vector2::getY() {
	return y;
}

double Vector2::getMagnitude() {
	return sqrt((x*x) + (y*y));
}
/**
 * Get the angle in radians between the vector and
 * the positive x-axis
 * 
 * @return the angle in radians
*/
double Vector2::getAngle() {
	return atan2(y, x);
}

/**
 * Return a normalized (magnitude = 1) version of this vector
*/
Vector2 Vector2::normalize() {
	double divisor = this->getMagnitude();
	double nx = x/divisor;
	double ny = y/divisor;
	return Vector2(nx, ny);
}

// ----------------- VirtualEncoder Struct ----------------- //

/**
 * Create a virtual tracking wheel encoder
 * 
 * @param _offset the perpendicular distance, in inches, between the tracking
 *                wheel and tracking centre
 * @param _lateral true if the wheel is perpendicular to the chassis' forward
*/
VirtualEncoder::VirtualEncoder(double _offset, bool _lateral) {
    this->offset = _offset;
    this->lateral = _lateral;
}

VirtualEncoder::VirtualEncoder() {}

long int VirtualEncoder::read() {
    return this->ticks;
}

void VirtualEncoder::reset() {
    this->ticks = 0;
}

/**
 * Update the encoder reading after a motion
 * 
 * @param dP the change in chassis position
 * @param dO the change in chassis orientation
*/
void VirtualEncoder::update(Vector2 dP, double dO) {
	if(dO == 0) {
		this->ticks += (lateral ? dP.getX() : dP.getY()) * TRACKING_WHEEL_INCH_TO_DEGREE;
		return;
	}

	// Solve odometry equations for dR/dL, then convert and increment
	Vector2 disp = rotateVector(dP, -dO/2);
	double newX = disp.getX() / 2 / sin(dO/2);
	double newY = disp.getY() / 2 / sin(dO/2);
	double dist = lateral ? newX : newY;
	this->ticks += ((dist + offset) * dO) * TRACKING_WHEEL_INCH_TO_DEGREE;
}
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
	lLast = 0;
	rLast = 0;
	bLast = 0;
	float x = trackingData.getX();
	float y = trackingData.getY();
	float left = 0;
	float right = 0;
	float lateral = 0;
	float angle = 0;

	printf("Start X: %f   Start Y: %f", x, y);

	// Start tracking loop
	while(1) {
		float localX, localY = 0;

		// Get encoder data
		float leftEncoder = leftTrackingWheel.read();
		float rightEncoder = rightTrackingWheel.read();
		float backEncoder = backTrackingWheel.read();

		// Calculate deltas
		lDelta = leftEncoder - lLast;
		rDelta = rightEncoder - rLast;
		bDelta = backEncoder - bLast;
		lDist = lDelta * TRACKING_WHEEL_DEGREE_TO_INCH;
		rDist = rDelta * TRACKING_WHEEL_DEGREE_TO_INCH;
		bDist = bDelta * TRACKING_WHEEL_DEGREE_TO_INCH;

		// Store readings for next deltas
		lLast = leftEncoder;
		rLast =	rightEncoder;
		bLast = backEncoder;

		// Increment persistent variables
		left += lDist;
		right += rDist;
		lateral += bDist;

		// Calculate arc angle
		float holdAngle = angle;
		angle = (right - left)/(lrOffset * 2.0f);
		aDelta = angle - holdAngle;

		// If theres an arc
		if(aDelta) {
			float radius = (rDist / aDelta) - lrOffset;
			halfA = aDelta/2.0f;
			float sinHalf = sin(halfA);
			localY = (radius * sinHalf) * 2.0f;

			float backRadius = (bDist / aDelta) - BACK_WHEEL_OFFSET;
			localX = (backRadius * sinHalf) * 2.0f;
		}
		// If no arc
		else {
			halfA = 0;
			aDelta = 0;
			localY = (rDist+lDist)/2;
			localX = bDist;
		}

		float p = -(halfA + holdAngle); // The global ending angle of the robot
		float cosP = cos(p);
		float sinP = sin(p);

		// Update the global position
		y += (localY * cosP) - (localX * sinP);
		x += (localY * sinP) + (localX * cosP);

		trackingData.update(x, y, angle);

		if (logPosition) {
			printf("X: %f Y: %f A: %f\n",
				roundUp(trackingData.getX(), 2),
				roundUp(trackingData.getY(), 2),
				radToDeg(roundUp(trackingData.getHeading(), 2))
			);
		}

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

Vector2 closest(Vector2 current, Vector2 head, Vector2 target) {
  Vector2 n = head.normalize();
  Vector2 v = target - current;
  double d = dot(v, n);
  return current + (n * d);
}

Vector2 closest(Vector2 current, Vector2 target) {
  return closest(current,
                 Vector2(sin(current.getAngle()), cos(current.getAngle())),
                 target);
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

double angleToPoint(Vector2 v1) {
	return (
		atan2(
			v1.getX() - trackingData.getX(),
			v1.getY() - trackingData.getY())
		) 
		- trackingData.getHeading();
}

double distanceToPoint(Vector2 v1, Vector2 v2) {
	double xDiff = v2.getX() - v1.getX();
	double yDiff = v2.getY() - v1.getY();
	return std::sqrt(std::pow(xDiff, 2) + std::pow(yDiff, 2));
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
		this->ticks += (lateral ? dP.getX() : dP.getY()) * DRIVE_DEGREE_TO_INCH;
		return;
	}

	// Solve odometry equations for dR/dL, then convert and increment
	Vector2 disp = rotateVector(dP, -dO/2);
	double newX = disp.getX() / 2 / sin(dO/2);
	double newY = disp.getY() / 2 / sin(dO/2);
	double dist = lateral ? newX : newY;
	this->ticks += ((dist + offset) * dO) * TRACKING_WHEEL_INCH_TO_DEGREE;
}
#ifndef _TRACKING_H_
#define _TRACKING_H_

#include "math.h"
#define _USE_MATH_DEFINES
#include "odom.h"

#define WHEELBASE 10.25
#define BACK_WHEEL_OFFSET 5

#define TRACKING_WHEEL_DIAMETER 2.75f
#define TRACKING_WHEEL_DEGREE_TO_INCH (M_PI * TRACKING_WHEEL_DIAMETER / 360)

double radToDeg(double r);
double degToRad(double d);

typedef struct Vector2 {
private:
	double x, y;

public:
	Vector2(double _x, double _y);
	Vector2();

	double getX();
	double getY();

	double getMagnitude();
	double getAngle();

	Vector2 normalize();

	// friend keyword allows access to private members
	friend Vector2 operator + (const Vector2 &v1, const Vector2 &v2);
	friend Vector2 operator - (const Vector2 &v1, const Vector2 &v2);
	friend Vector2 operator * (const Vector2 &v1, double scalar);
} Vector2;

typedef struct TrackingData {
private:
	bool suspendModulus = false;
	double heading;
	Vector2 pos;

public:
	TrackingData(double _x, double _y, double _h);

	double getX();
	double getY();
	double getHeading();
	Vector2 getPos();
	Vector2 getForward();

	void suspendAngleModulus();
	void resumeAngleModulus();

	void update(double _x, double _y, double _h);

} TrackingData;

/*
	Tracks encoder positions; No parameters
*/
void tracking();
Vector2 rotateVector(Vector2 vec, double angle);
Vector2 toLocalCoordinates(Vector2 vec);
Vector2 toGlobalCoordinates(Vector2 vec);
double dot(Vector2 v1, Vector2 v2);
Vector2 closest(Vector2 current, Vector2 head, Vector2 target);
Vector2 closest(Vector2 current, Vector2 target);
double angleToPoint(Vector2 v1);
double distanceToPoint(Vector2 v1, Vector2 v2);
extern TrackingData trackingData;

// Simulate the chassis' tracking wheels, replaces pros::ADIEncoder
typedef struct VirtualEncoder {
private:
	// State
    double ticks;
    double offset; // Positive = right
    bool lateral;

public:
	// Constructors
    VirtualEncoder(double _offset, bool _lateral = false);
	VirtualEncoder();

	// Interface
    void reset();
    long int read();
    void update(Vector2 dP, double dO); // params in local space

} VirtualEncoder;

Vector2 glmToCustom(glm::vec2 v);
glm::vec2 customToGLM(Vector2 v);

extern VirtualEncoder leftTrackingWheel;
extern VirtualEncoder rightTrackingWheel;
extern VirtualEncoder backTrackingWheel;
extern XDrive chassis;

extern bool logPosition;

#endif

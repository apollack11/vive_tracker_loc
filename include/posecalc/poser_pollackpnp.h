// opencv_pose_calc.h
#ifndef POSER_POLLACKPNP_H
#define POSER_POLLACKPNP_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
        float Pos[3];
        float SE3Mat[16];
} Pose;

typedef struct
{
	float x;
	float y;
	float z;
} Point;

// used to store info about each sensor
typedef struct
{
	Point point; // location of the sensor on the tracked object;
	Point normal; // unit vector indicating the normal for the sensor
	double theta; // "horizontal" angular measurement from lighthouse radians
	double phi; // "vertical" angular measurement from lighthouse in radians.
	int id;
} TrackedSensor;

// used to store info about all the sensors
typedef struct
{
	size_t numSensors;
	TrackedSensor sensor[0];
} TrackedObject;

void PoseCalculation(TrackedObject *to, Pose *pose);

#ifdef __cplusplus
    }
#endif

#endif

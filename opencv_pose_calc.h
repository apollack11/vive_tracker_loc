// opencv_pose_calc.h
#ifndef OPENCV_POSE_CALC_H
#define OPENCV_POSE_CALC_H

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

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

void PoseCalculation(TrackedObject *to);

#ifdef __cplusplus
    }
#endif

#endif

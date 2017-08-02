#include <survive.h>
#include <stdio.h>
#include <stdlib.h>
#include "linmath.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <dclapack.h>

// QUESTION: what do I still need from this?
typedef struct
{
#define OLD_ANGLES_BUFF_LEN 3
	FLT oldAngles[SENSORS_PER_OBJECT][2][NUM_LIGHTHOUSES][OLD_ANGLES_BUFF_LEN]; // sensor, sweep axis, lighthouse, instance
	int angleIndex[NUM_LIGHTHOUSES][2]; // index into circular buffer ahead. separate index for each axis.
	int lastAxis[NUM_LIGHTHOUSES];
} PollackRadiiData;

#define MAX_POINT_PAIRS 100

// defining SQUARED as a function
#define SQUARED(x) ((x)*(x))

typedef struct
{
	FLT x;
	FLT y;
	FLT z;
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

// used to store pairs of sensors
// and the angle and distance between them
typedef struct
{
	Point a;
	Point b;
	FLT alpha;
	FLT distance;
	char ai;
	char bi;
} PointsAndAngle;

void inverseMatrix3by3(Matrix3x3 m, Matrix3x3 result)
{
	FLT det = m.val[0][0] * (m.val[1][1]*m.val[2][2] - m.val[2][1]*m.val[1][2]) \
	 				- m.val[0][1] * (m.val[1][0]*m.val[2][2] - m.val[1][2]*m.val[2][0]) \
					+ m.val[0][2] * (m.val[1][0]*m.val[2][1] - m.val[1][1]*m.val[2][0]);

	if (det == 0)
	{
		printf("ERROR: Determinant of the matrix is 0\n");
		return;
	}
	FLT invdet = 1 / det;

	result.val[0][0] = (m.val[1][1]*m.val[2][2] - m.val[2][1]*m.val[1][2]) * invdet;
	result.val[0][1] = (m.val[0][2]*m.val[2][1] - m.val[0][1]*m.val[2][2]) * invdet;
	result.val[0][2] = (m.val[0][1]*m.val[1][2] - m.val[0][2]*m.val[1][1]) * invdet;
	result.val[1][0] = (m.val[1][2]*m.val[2][0] - m.val[1][0]*m.val[2][2]) * invdet;
	result.val[1][1] = (m.val[0][0]*m.val[2][2] - m.val[0][2]*m.val[2][0]) * invdet;
	result.val[1][2] = (m.val[1][0]*m.val[0][2] - m.val[0][0]*m.val[1][2]) * invdet;
	result.val[2][0] = (m.val[1][0]*m.val[2][1] - m.val[2][0]*m.val[1][1]) * invdet;
	result.val[2][1] = (m.val[2][0]*m.val[0][1] - m.val[0][0]*m.val[2][1]) * invdet;
	result.val[2][2] = (m.val[0][0]*m.val[1][1] - m.val[1][0]*m.val[0][1]) * invdet;

	PRINT(result.val, 3, 3);
}

// calculate distance between two points
// used for to calculate distance between sensors
static FLT distance(Point a, Point b)
{
	FLT x = a.x - b.x;
	FLT y = a.y - b.y;
	FLT z = a.z - b.z;
	return FLT_SQRT(x*x + y*y + z*z);
}

// finds the angle between two sensors on the tracker
// QUESTION: is this correct??? pretty sure it is
FLT angleBetweenSensorPair(TrackedSensor *a, TrackedSensor *b)
{
	FLT angle = FLT_ACOS(FLT_COS(a->phi - b->phi)*FLT_COS(a->theta - b->theta));

	// This was from tanay's -- it includes the COS() around the angle
	// I don't think this works...
	// FLT angle = FLT_SIN(a->phi)*FLT_COS(a->theta)*FLT_SIN(b->phi)*FLT_COS(b->theta) + FLT_SIN(a->phi)*FLT_SIN(a->theta)*FLT_SIN(b->phi)*FLT_SIN(b->theta) + FLT_COS(a->phi)*FLT_COS(b->phi);

	return angle;
}



static void SolveForRadii(FLT *objPosition, FLT *objOrientation, TrackedObject *to)
{
	PointsAndAngle pna[MAX_POINT_PAIRS];

	size_t pnaCount = 0;
	for (unsigned int i = 0; i < to->numSensors; i++)
	{
		for (unsigned int j = 0; j < i; j++)
		{
			pna[pnaCount].a = to->sensor[i].point;
			pna[pnaCount].b = to->sensor[j].point;

			pna[pnaCount].alpha = angleBetweenSensorPair(&to->sensor[i], &to->sensor[j]);
			pna[pnaCount].distance = distance(pna[pnaCount].a, pna[pnaCount].b);
			pna[pnaCount].ai = i;
			pna[pnaCount].bi = j;

			pnaCount++;
		}
	}

	printf("pnaCount: %zd\n", pnaCount);
	for (unsigned int i = 0; i < pnaCount; i++)
	{
		printf("pna[%d]: alpha = %f, distance = %f\n", i, pna[i].alpha, pna[i].distance);
	}

	Matrix3x3 jacobian;
	Matrix3x3 inverseJacobian;
	FLT f[3][1];
	FLT grad[3][1] = {100, 100, 100};
	FLT R[3] = {50, 51, 52};
	FLT AB = pna[0].distance; // distances are accurate (measured with calipers)
	FLT BC = pna[1].distance;
	FLT AC = pna[2].distance;
	FLT alphaAB = pna[0].alpha; // angles are accurate (measured and calculated)
	FLT alphaBC = pna[1].alpha;
	FLT alphaAC = pna[2].alpha;

	int iterations = 0;

	printf("ENTERING WHILE LOOP\n");

	while (grad[1][0] > 0.01 || grad[2][0] > 0.01 || grad[3][0] > 0.01)
	{
		jacobian.val[0][0] = 2*R[0] - 2*R[1] * FLT_COS(alphaAB);
		jacobian.val[0][1] = 2*R[1] - 2*R[0] * FLT_COS(alphaAB);
		jacobian.val[0][2] = 0;

		jacobian.val[1][0] = 0;
		jacobian.val[1][1] = 2*R[1] - 2*R[2] * FLT_COS(alphaBC);
		jacobian.val[1][2] = 2*R[2] - 2*R[1] * FLT_COS(alphaBC);

		jacobian.val[2][0] = 2*R[0] - 2*R[2] * FLT_COS(alphaAC);
		jacobian.val[2][1] = 0;
		jacobian.val[2][2] = 2*R[2] - 2*R[0] * FLT_COS(alphaAC);

		INV(jacobian.val, inverseJacobian.val, 3);

		f[0][0] = SQUARED(R[0]) + SQUARED(R[1]) - 2*R[0]*R[1]*FLT_COS(alphaAB) - SQUARED(AB);
		f[1][0] = SQUARED(R[1]) + SQUARED(R[2]) - 2*R[1]*R[2]*FLT_COS(alphaBC) - SQUARED(BC);
		f[2][0] = SQUARED(R[0]) + SQUARED(R[2]) - 2*R[0]*R[2]*FLT_COS(alphaAC) - SQUARED(AC);

		MUL(inverseJacobian.val, f, grad, 3, 3, 1);

		R[0] -= grad[0][0];
		R[1] -= grad[1][0];
		R[2] -= grad[2][0];

		iterations++;
	}

	printf("total iterations: %d\n", iterations);

	for (unsigned int i = 0; i < 3; i++)
	{
		printf("grad[%d] = %f\n", i, grad[i][0]);
	}

	printf("Ra: %f\n", R[0]);
	printf("Rb: %f\n", R[1]);
	printf("Rc: %f\n", R[2]);

	FLT avgX = 0;
	FLT avgY = 0;
	FLT avgZ = 0;
	for (int i = 0; i < to->numSensors; i++)
	{
		FLT x = R[i]*FLT_SIN(to->sensor[i].phi)*FLT_COS(to->sensor[i].theta) - to->sensor[i].point.x;
		FLT y = R[i]*FLT_SIN(to->sensor[i].phi)*FLT_SIN(to->sensor[i].theta) - to->sensor[i].point.y;
		FLT z = R[i]*FLT_COS(to->sensor[i].phi);
		printf("location[%d]: (%f, %f, %f)\n", i, x, y, z);
		avgX += x;
		avgY += y;
		avgZ += z;
	}
	avgX /= 3;
	avgY /= 3;
	avgZ /= 3;

	printf("Average Location: (%f, %f, %f)\n\n", avgX, avgY, avgZ);

}




static void QuickPose(SurviveObject *so)
{
	// get the poser data from SurviveObject
	PollackRadiiData * prd = so->PoserData;

	// if (! so->ctx->bsd[0].OOTXSet)
	// {
	// 	// we don't know where we are!  Augh!!!
	// 	return;
	// }

	TrackedObject * to;

	to = malloc(sizeof(TrackedObject) + (SENSORS_PER_OBJECT * sizeof(TrackedSensor)));

	int sensorCount = 0;

	// so->nr_locations is the number of sensors on the current device
	// should be 22 for the Vive Tracker
	for (int i = 0; i < so->nr_locations; i++)
	{
		int lh = 0;

		// prd->angleIndex[lh][0 or 1] angleIndex for each axis of the given lighthouse
		// OLD_ANGLES_BUFF_LEN is 3
		int angleIndex0 = (prd->angleIndex[lh][0] + 1 + OLD_ANGLES_BUFF_LEN) % OLD_ANGLES_BUFF_LEN;
		int angleIndex1 = (prd->angleIndex[lh][1] + 1 + OLD_ANGLES_BUFF_LEN) % OLD_ANGLES_BUFF_LEN;

		// prd->oldAngles gives the angle seen by a sensor given [sensor_id][axis][lh][angleIndex]
		// check if the sensor has an estimated angle not equal to 0
		if ((prd->oldAngles[i][0][lh][angleIndex0] != 0 && prd->oldAngles[i][1][lh][angleIndex1] != 0))
		{
			// get the normal vector and x,y,z location of current sensor
			FLT norm[3] = { so->sensor_normals[i * 3 + 0] , so->sensor_normals[i * 3 + 1] , so->sensor_normals[i * 3 + 2] };
			FLT point[3] = { so->sensor_locations[i * 3 + 0] , so->sensor_locations[i * 3 + 1] , so->sensor_locations[i * 3 + 2] };

			// TODO: remove these two lines!!!
			//quatrotatevector(norm, downQuat, norm);
			//quatrotatevector(point, downQuat, point);

			// store normal vector and location in TrackedObject
			// each sensor that sees something has information stored
			to->sensor[sensorCount].normal.x = norm[0];
			to->sensor[sensorCount].normal.y = norm[1];
			to->sensor[sensorCount].normal.z = norm[2];
			to->sensor[sensorCount].point.x = point[0];
			to->sensor[sensorCount].point.y = point[1];
			to->sensor[sensorCount].point.z = point[2];
			to->sensor[sensorCount].theta = prd->oldAngles[i][0][lh][angleIndex0]; // lighthouse 0, angle 0 (horizontal)
			to->sensor[sensorCount].phi = prd->oldAngles[i][1][lh][angleIndex1]; // lighthouse 0, angle 1 (vertical)

			// FOR DEBUGGING ANGLES
			printf("sensor[%d] theta: %f\n", sensorCount, to->sensor[sensorCount].theta);
			printf("sensor[%d] phi: %f\n", sensorCount, to->sensor[sensorCount].phi);

			sensorCount++;
		}
	}
	// sensorCount is the number of sensors that did not have an angle of 0
	// store this in the TrackedObject
	to->numSensors = sensorCount;

	// good to check number of sensors being seen
	printf("sensorCount: %zd\n", to->numSensors);

	if (to->numSensors == 3)
	{
		FLT pos[3];
		FLT orient[4];
		SolveForRadii(pos, orient, to);
	}
	else
	{
		printf("WRONG AMOUNT OF SENSORS (%zd)\n", to->numSensors);
	}

	free(to);
}



int PoserPollackRadii( SurviveObject * so, PoserData * pd )
{
	PoserType pt = pd->pt;
	SurviveContext * ctx = so->ctx;
	PollackRadiiData * prd = so->PoserData;

	if( !prd ) so->PoserData = prd = malloc( sizeof( PollackRadiiData ) );

	switch( pt )
	{
	case POSERDATA_IMU:
	{
		PoserDataIMU * imu = (PoserDataIMU*)pd;
		//printf( "IMU:%s (%f %f %f) (%f %f %f)\n", so->codename, imu->accel[0], imu->accel[1], imu->accel[2], imu->gyro[0], imu->gyro[1], imu->gyro[2] );
		break;
	}
	case POSERDATA_LIGHT:
	{
		PoserDataLight * l = (PoserDataLight*)pd;
		//printf( "LIG:%s %d @ %f rad, %f s (AC %d) (TC %d)\n", so->codename, l->sensor_id, l->angle, l->length, l->acode, l->timecode );

		int axis = l->acode & 0x1;

		if ((prd->lastAxis[l->lh] != (l->acode & 0x1)))
		{
			// only once per full cycle
			if (0 == l->lh && axis)
			{
				static unsigned int counter = 1;

				counter++;

				if (counter % 4 == 0)
				{
					// MAIN CALCULATIONS
					QuickPose(so);
				}
			}

			// axis changed, time to increment the circular buffer index.
			prd->angleIndex[l->lh][axis]++;
			prd->angleIndex[l->lh][axis] = prd->angleIndex[l->lh][axis] % OLD_ANGLES_BUFF_LEN;

			// and clear out the data.
			for (int i=0; i < SENSORS_PER_OBJECT; i++)
			{
				prd->oldAngles[i][axis][l->lh][prd->angleIndex[l->lh][axis]] = 0;
			}

			prd->lastAxis[l->lh] = axis;
		}

		prd->oldAngles[l->sensor_id][axis][l->lh][prd->angleIndex[l->lh][axis]] = l->angle;
		break;
	}
	case POSERDATA_FULL_SCENE:
	{
		PoserDataFullScene * fs = (PoserDataFullScene*)pd;
		//printf( "Full scene data.\n" );
		break;
	}
	case POSERDATA_DISASSOCIATE:
	{
		free( prd );
		so->PoserData = 0;
		//printf( "Need to disassociate.\n" );
		break;
	}
	}
	return 0;
}


REGISTER_LINKTIME( PoserPollackRadii );

#include <survive.h>
#include <stdio.h>
#include <stdlib.h>
#include "linmath.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <dclapack.h>
#include "opencv_pose_calc.h"

// QUESTION: what do I still need from this?
typedef struct
{
#define OLD_ANGLES_BUFF_LEN 3
	FLT oldAngles[SENSORS_PER_OBJECT][2][NUM_LIGHTHOUSES][OLD_ANGLES_BUFF_LEN]; // sensor, sweep axis, lighthouse, instance
	int angleIndex[NUM_LIGHTHOUSES][2]; // index into circular buffer ahead. separate index for each axis.
	int lastAxis[NUM_LIGHTHOUSES];
} PollackPnPData;

#define MAX_POINT_PAIRS 100

// defining SQUARED as a function
#define SQUARED(x) ((x)*(x))

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

// multiply 2 matrices in 1D arrays
void multiplyMatrices(float* matA, int rA, int cA, float* matB, int rB, int cB, float* matC, int rC, int cC) 
{
    for (int i = 0; i < rA; i++) 
    {
        for (int j = 0; j < cB; j++) 
	{
            float sum = 0.0;
            for (int k = 0; k < rB; k++)
                sum = sum + matA[i * cA + k] * matB[k * cB + j];
            matC[i * cC + j] = sum;
        }
    }
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


static void QuickPose(SurviveObject *so)
{
	// get the poser data from SurviveObject
	PollackPnPData * ppd = so->PoserData;

	// only start polling for angles after receiving the first full packet
	if (! so->ctx->bsd[0].OOTXSet)
	{
		return;
	}

	TrackedObject * to;

	to = malloc(sizeof(TrackedObject) + (SENSORS_PER_OBJECT * sizeof(TrackedSensor)));

	int sensorCount = 0;

	// so->nr_locations is the number of sensors on the current device
	// should be 22 for the Vive Tracker
	for (int i = 0; i < so->nr_locations; i++)
	{
		int lh = 0;

		// ppd->angleIndex[lh][0 or 1] angleIndex for each axis of the given lighthouse
		// OLD_ANGLES_BUFF_LEN is 3
		int angleIndex0 = (ppd->angleIndex[lh][0] + 1 + OLD_ANGLES_BUFF_LEN) % OLD_ANGLES_BUFF_LEN;
		int angleIndex1 = (ppd->angleIndex[lh][1] + 1 + OLD_ANGLES_BUFF_LEN) % OLD_ANGLES_BUFF_LEN;

		// ppd->oldAngles gives the angle seen by a sensor given [sensor_id][axis][lh][angleIndex]
		// check if the sensor has an estimated angle not equal to 0
		if ((ppd->oldAngles[i][0][lh][angleIndex0] != 0 && ppd->oldAngles[i][1][lh][angleIndex1] != 0))
		{
			// get the normal vector and x,y,z location of current sensor
			FLT norm[3] = { so->sensor_normals[i * 3 + 0] , so->sensor_normals[i * 3 + 1] , so->sensor_normals[i * 3 + 2] };
			FLT point[3] = { so->sensor_locations[i * 3 + 0] , so->sensor_locations[i * 3 + 1] , so->sensor_locations[i * 3 + 2] };

			// store normal vector and location in TrackedObject
			// each sensor that sees something has information stored
			to->sensor[sensorCount].normal.x = norm[0];
			to->sensor[sensorCount].normal.y = norm[1];
			to->sensor[sensorCount].normal.z = norm[2];
			to->sensor[sensorCount].point.x = point[0];
			to->sensor[sensorCount].point.y = point[1];
			to->sensor[sensorCount].point.z = point[2];
			to->sensor[sensorCount].theta = ppd->oldAngles[i][0][lh][angleIndex0]; // lighthouse 0, angle 0 (horizontal)
			to->sensor[sensorCount].phi = ppd->oldAngles[i][1][lh][angleIndex1]; // lighthouse 0, angle 1 (vertical)

			sensorCount++;
		}
	}

	// sensorCount is the number of sensors that did not have an angle of 0
	// store this in the TrackedObject
	to->numSensors = sensorCount;

	// good to check number of sensors being seen
	/* printf("sensorCount: %zd\n", to->numSensors); */

	Pose *pose;
	pose = malloc(sizeof(Pose));
	if (sensorCount > 4)
	{
	  PoseCalculation(to, pose);
	  
	  // Store tracked object pose in SurviveObject
      	  so->FromLHPose[0].Pos[0] = pose->Pos[0];
	  so->FromLHPose[0].Pos[1] = pose->Pos[1];
	  so->FromLHPose[0].Pos[2] = pose->Pos[2];

	  FLT quat[4];
	  FLT matrix44[16];

	  for (int i = 0; i < 16; i++)
	  {
	    matrix44[i] = pose->SE3Mat[i];
	  }
	  FLT rot90[16];
	  for (int i = 0; i < 16; i++)
	  {
	    rot90[i] = 0;
	  }
	  rot90[0] = 1;
	  rot90[6] = -1;
	  rot90[9] = 1;

	  FLT newMat[16];

	  multiplyMatrices(matrix44, 4, 4, rot90, 4, 4, newMat, 4, 4);

	  /* matrix44[0]*rot90[0] + matrix44[1]*rot90[4] + matrix44[2]*rot90[8] + matrix44[3]*rot90[12] */
	  /* matrix44[0]*rot90[1] + matrix44[1]*rot90[5] + matrix44[2]*rot90[9] + matrix44[3]*rot90[13] */
	  /* matrix44[0]*rot90[2] + matrix44[1]*rot90[6] + matrix44[2]*rot90[10] + matrix44[3]*rot90[14] */
	  /* matrix44[0]*rot90[3] + matrix44[1]*rot90[7] + matrix44[2]*rot90[11] + matrix44[3]*rot90[15] */
	  
	  /* matrix44[4]*rot90[0] + matrix44[5]*rot90[4] + matrixx44[6]*rot90[8] + matrix44[7]*rot90[12] */

	  quatfrommatrix(quat, matrix44);

	  so->FromLHPose[0].Rot[0] = quat[3];
	  so->FromLHPose[0].Rot[1] = quat[1];
	  so->FromLHPose[0].Rot[2] = quat[2];
	  so->FromLHPose[0].Rot[3] = quat[0];
	}
	
	free(to);
}



int PoserPollackPnP( SurviveObject * so, PoserData * pd )
{
	PoserType pt = pd->pt;
	SurviveContext * ctx = so->ctx;
	PollackPnPData * ppd = so->PoserData;

	if( !ppd ) so->PoserData = ppd = malloc( sizeof( PollackPnPData ) );

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

		if ((ppd->lastAxis[l->lh] != (l->acode & 0x1)))
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
			ppd->angleIndex[l->lh][axis]++;
			ppd->angleIndex[l->lh][axis] = ppd->angleIndex[l->lh][axis] % OLD_ANGLES_BUFF_LEN;

			// and clear out the data.
			for (int i=0; i < SENSORS_PER_OBJECT; i++)
			{
				ppd->oldAngles[i][axis][l->lh][ppd->angleIndex[l->lh][axis]] = 0;
			}

			ppd->lastAxis[l->lh] = axis;
		}

		ppd->oldAngles[l->sensor_id][axis][l->lh][ppd->angleIndex[l->lh][axis]] = l->angle;
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
		free( ppd );
		so->PoserData = 0;
		//printf( "Need to disassociate.\n" );
		break;
	}
	}
	return 0;
}


REGISTER_LINKTIME( PoserPollackPnP );

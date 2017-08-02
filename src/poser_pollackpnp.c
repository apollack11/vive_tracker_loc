#include <survive.h>
#include <stdio.h>
#include <stdlib.h>
#include "linmath.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <dclapack.h>
#include <cv.h>
#include <highgui.h>

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
			to->sensor[sensorCount].theta = ppd->oldAngles[i][0][lh][angleIndex0]; // lighthouse 0, angle 0 (horizontal)
			to->sensor[sensorCount].phi = ppd->oldAngles[i][1][lh][angleIndex1]; // lighthouse 0, angle 1 (vertical)

			// FOR DEBUGGING ANGLES
			printf("sensor[%d] theta: %f\n", sensorCount, to->sensor[sensorCount].theta);
			printf("sensor[%d] phi: %f\n", sensorCount, to->sensor[sensorCount].phi);
			printf("sensor[%d] location: (%f, %f, %f)\n", sensorCount, to->sensor[sensorCount].point.x, to->sensor[sensorCount].point.y, to->sensor[sensorCount].point.z);

			sensorCount++;
		}
	}
	// sensorCount is the number of sensors that did not have an angle of 0
	// store this in the TrackedObject
	to->numSensors = sensorCount;

	// good to check number of sensors being seen
	printf("sensorCount: %zd\n", to->numSensors);

	// IplImage* img = 0;
	// img=cvLoadImage("apple.png",0);
	// cvShowImage("mainWin", img );
	// cvWaitKey(0);
	// cvReleaseImage(&img );







	// //some variables
	// CameraCalibrationData* calibrationData;
	// calibrationData->enable = false;
	//
	// IplImage*    originalImage;
	// IplImage*    grayImage;
	//
	// // number of inner corners
	// CvSize chessBoardSize;
	// chessBoardSize.width = 5;
	// chessBoardSize.height = 4;
	//
	// float chessBoardSquareInMeters = 0.03f;
	//
	// // detected points in image
	// std::vector<CvPoint2D32f> chessBoardPoints;


	if (sensorCount > 4) {
		CvMat* cameraMatrix = cvCreateMat(3, 3, CV_64F);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				CV_MAT_ELEM(*cameraMatrix, float, i, j) = 0.0f;
			}
		}
		CV_MAT_ELEM(*cameraMatrix, float, 0, 0) = 1.0f;
		CV_MAT_ELEM(*cameraMatrix, float, 1, 1) = 1.0f;
		CV_MAT_ELEM(*cameraMatrix, float, 2, 2) = 1.0f;

		CvMat* distorsionCoeffs = cvCreateMat(1, 5, CV_64F);
		for (int i = 0; i < 1; i++)
		{
			for (int j = 0; j < 5; j++)
			{
				CV_MAT_ELEM(*distorsionCoeffs, float, i, j) = 0.0f;
			}
		}

		CvMat* objectPoints = cvCreateMat(sensorCount, 3, CV_64F);
		for (int i = 0; i < sensorCount; i++)
		{
			CV_MAT_ELEM(*objectPoints, float, i, 0) = (float)to->sensor[i].point.x;
			CV_MAT_ELEM(*objectPoints, float, i, 1) = (float)to->sensor[i].point.y;
			CV_MAT_ELEM(*objectPoints, float, i, 2) = (float)to->sensor[i].point.z;
		}

		CvMat* imagePoints = cvCreateMat(sensorCount, 2, CV_64F);
		for (int i = 0; i < sensorCount; i++)
		{
			CV_MAT_ELEM(*imagePoints, float, i, 0) = FLT_TAN(to->sensor[i].phi);
			CV_MAT_ELEM(*imagePoints, float, i, 1) = FLT_TAN(to->sensor[i].theta);
		}

		CvMat* rotationVector = cvCreateMat(1, 3, CV_64F);
		CvMat* translationVector = cvCreateMat(1, 3, CV_64F);

		cvFindExtrinsicCameraParams2(objectPoints, imagePoints, cameraMatrix, distorsionCoeffs, rotationVector, translationVector, 0);

		printf("ROTATION[0][0]: %f\n", CV_MAT_ELEM(*rotationVector, float, 0, 0));
		printf("ROTATION[0][1]: %f\n", CV_MAT_ELEM(*rotationVector, float, 0, 1));
		printf("ROTATION[0][2]: %f\n", CV_MAT_ELEM(*rotationVector, float, 0, 2));
		printf("TRANSLATION[0][0]: %f\n", CV_MAT_ELEM(*translationVector, float, 0, 0));
		printf("TRANSLATION[0][1]: %f\n", CV_MAT_ELEM(*translationVector, float, 0, 1));
		printf("TRANSLATION[0][2]: %f\n", CV_MAT_ELEM(*translationVector, float, 0, 2));

	  //  // process results
	  //  CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64F);
	  //  cvRodrigues2(rotationVector, rotationMatrix, 0);
	  //  float temp;
		//  FLT rotationMat[3][3];
	  //  for( int i = 0; i < 3; i++ )
	  //  {
	  //      for( int j = 0; j < 3; j++ )
	  //      {
	  //          temp = CV_MAT_ELEM(*rotationMatrix, float, i, j);
	  //          assert(!cvIsNaN(temp));
	  //          rotationMat[i][j] = temp;
	  //      }
	  //  }

	}



	//      // process results
	//      CvMat* rotationMatrix = cvCreateMat(3, 3, CV_32FC1);
	//      cvRodrigues2(rotationVector, rotationMatrix, 0);
	//      float temp;
	//      for( int i = 0; i < 3; i++ )
	//      {
	//          for( int j = 0; j < 3; j++ )
	//          {
	//              temp = CV_MAT_ELEM(*rotationMatrix, float, i, j);
	//              assert(!cvIsNaN(temp));
	//              cameraTransformationOutput[i][j] = temp;
	//          }
	//      }
	//      float translationVectorLength = 0.0f;
	//      for( int i = 0; i < 3; i++ )
	//      {
	//          temp = CV_MAT_ELEM(*translationVector, float, 0, i);
	//          assert(!cvIsNaN(temp));
	//
	//          // multiply units to get meters in real world
	//          temp *= chessBoardSquareInMeters;
	//
	//          cameraTransformationOutput[3][i] = temp;
	//          translationVectorLength += temp*temp;
	//      }
	//      translationVectorLength = sqrt(translationVectorLength);
	//      for( int i = 0; i < 3; i++ )
	//      {
	//          cameraTransformationOutput[i][3] = 0.0f;
	//      }
	//      cameraTransformationOutput[3][3] = 1.0f;
	//
	//      cvReleaseMat(&rotationMatrix);
	//      cvReleaseMat(&rotationVector);
	//      cvReleaseMat(&translationVector);
	//      cvReleaseMat(&distorsionCoeffs);
	//      cvReleaseMat(&intrinsicMatrix);
	//      cvReleaseMat(&imagePoints);
	//      cvReleaseMat(&objectPoints);
	//
	//      return true;
	// }




















	// ///////
	// // Trying to get OpenCV C++ implementation working
	// ///////
	//
	// // initialize vectors to store image and object points
	// std::vector<cv::Point2f> imagePoints;
	// std::vector<cv::Point3f> objectPoints;
	//
	// // populate image points and object points with image location and body coordinates from each sensor
	// for (int i = 0; i < to->numSensors; i++)
	// {
	// 	FLT x = to->sensor[i].point.x;
	// 	FLT y = to->sensor[i].point.y;
	// 	FLT z = to->sensor[i].point.z;
	//
	// 	objectPoints.push_back(cv::Point3f(x,y,z));
	//
	// 	FLT ix = FLT_TAN(to->sensor[i].phi);
	// 	FLT iy = FLT_TAN(to->sensor[i].theta);
	//
	// 	imagePoints.push_back(cv::Point2f(ix,iy));
	// }
	//
	// // initialize camera matrix as identity matrix
	// cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
	// cv::setIdentity(cameraMatrix);
	//
	// // intialize distortion coefficients to all zeros
	// cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
	// distCoeffs.at<double>(0) = 0;
	// distCoeffs.at<double>(1) = 0;
	// distCoeffs.at<double>(2) = 0;
	// distCoeffs.at<double>(3) = 0;
	//
	// // initialize OpenCV Mat for resulting rotation and translation vectors
	// cv::Mat rvec(3, 1, cv::DataType<double>::type);
	// cv::Mat tvec(3, 1, cv::DataType<double>::type);
	//
	// cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
	//
	// std::cout << "rvec: " << rvec << std::endl;
	// std::cout << "tvec: " << tvec << std::endl;

	// if (to->numSensors == 3)
	// {
	// 	FLT pos[3];
	// 	FLT orient[4];
	// 	SolveForRadii(pos, orient, to);
	// }
	// else
	// {
	// 	printf("WRONG AMOUNT OF SENSORS (%zd)\n", to->numSensors);
	// }

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


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>
#include <math.h>

struct Point {
	float x;
	float y;
	float z;
};

// used to store info about each sensor
struct TrackedSensor {
	Point point; // location of the sensor on the tracked object;
	Point normal; // unit vector indicating the normal for the sensor
	double theta; // "horizontal" angular measurement from lighthouse radians
	double phi; // "vertical" angular measurement from lighthouse in radians.
	int id;
};

// used to store info about all the sensors
struct TrackedObject {
	size_t numSensors;
	TrackedSensor sensor[0];
};

std::vector<cv::Point2f> GatherImagePoints(TrackedObject *to);
std::vector<cv::Point3f> GatherObjectPoints(TrackedObject *to);
void PopulateTrackedObject(TrackedObject *to);

int main( int argc, char* argv[])
{
  // used as a placeholder until I can pass TrackedObject in from Poser
  TrackedObject *to = new TrackedObject;
  PopulateTrackedObject(to);

  // Converts values from TrackedObject to vectors of imagePoints and objectPoints
  std::vector<cv::Point2f> imagePoints = GatherImagePoints(to);
  std::vector<cv::Point3f> objectPoints = GatherObjectPoints(to);

  // use identity matrix for cameraMatrix
  cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
  cv::setIdentity(cameraMatrix);

  // make distCoeffs Mat of 4 zeros
  cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
  distCoeffs.at<double>(0) = 0;
  distCoeffs.at<double>(1) = 0;
  distCoeffs.at<double>(2) = 0;
  distCoeffs.at<double>(3) = 0;

  // initialize rvec and tvec
  cv::Mat rvec(3,1,cv::DataType<double>::type);
  cv::Mat tvec(3,1,cv::DataType<double>::type);

  // solvePnP
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

  // print output vectors
  std::cout << "rvec: " << rvec << std::endl;
  std::cout << "tvec: " << tvec << std::endl;

  std::vector<cv::Point2f> projectedPoints;
  cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

  for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
    std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
    }

  return 0;
}

void PopulateTrackedObject(TrackedObject *to) {
  int sensorCount = 10;
  // to = malloc(sizeof(TrackedObject) + (sensorCount * sizeof(TrackedSensor)));

  for (int i = 0; i < sensorCount; i++) {
    to->sensor[i].point.x = rand()/double(RAND_MAX);
    to->sensor[i].point.y = rand()/double(RAND_MAX);
    to->sensor[i].point.z = rand()/double(RAND_MAX);
    to->sensor[i].theta = rand()/double(RAND_MAX);
    to->sensor[i].phi = rand()/double(RAND_MAX);
    to->numSensors++;
  }
}

std::vector<cv::Point2f> GatherImagePoints(TrackedObject *to)
{
  std::vector<cv::Point2f> points;

  for (int i = 0; i < to->numSensors; i++) {
    points.push_back(cv::Point2f(tan(to->sensor[i].phi), tan(to->sensor[i].theta)));
  }

  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }

  return points;
}


std::vector<cv::Point3f> GatherObjectPoints(TrackedObject *to)
{
  std::vector<cv::Point3f> points;

  for (int i = 0; i < to->numSensors; i++) {
    points.push_back(cv::Point3f(to->sensor[i].point.x, to->sensor[i].point.y, to->sensor[i].point.z));
  }

  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }

  return points;
}

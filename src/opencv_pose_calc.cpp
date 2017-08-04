#include "opencv_pose_calc.h"
#include <iostream>
#include <string>

std::vector<cv::Point2f> GatherImagePoints(TrackedObject *to);
std::vector<cv::Point3f> GatherObjectPoints(TrackedObject *to);
bool isRotationMatrix(cv::Mat &R);
std::vector<cv::Point3f> rotationMatrixToEulerAngles(cv::Mat &R);

void PoseCalculation(TrackedObject *to)
{
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
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, CV_EPNP);

  // convert rvec to roation matrix
  cv::Mat R(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(rvec, R);

  // convert rotation matrix to euler angles
  std::vector<cv::Point3f> angles = rotationMatrixToEulerAngles(R);

  // print output of location and pose
  std::cout << "Location (x, y, z): " << tvec << std::endl;
  std::cout << "Pose (roll, pitch, yaw): " << angles << std::endl;
}

std::vector<cv::Point2f> GatherImagePoints(TrackedObject *to)
{
  std::vector<cv::Point2f> points;

  for (int i = 0; i < to->numSensors; i++)
  {
    points.push_back(cv::Point2f(tan(to->sensor[i].phi), tan(to->sensor[i].theta)));
  }

  return points;
}


std::vector<cv::Point3f> GatherObjectPoints(TrackedObject *to)
{
  std::vector<cv::Point3f> points;

  for (int i = 0; i < to->numSensors; i++) {
    points.push_back(cv::Point3f(to->sensor[i].point.x, to->sensor[i].point.y, to->sensor[i].point.z));
  }
  
  return points;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
std::vector<cv::Point3f> rotationMatrixToEulerAngles(cv::Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    std::vector<cv::Point3f> angles;
    angles.push_back(cv::Point3f(x,y,z));
    return angles;
}

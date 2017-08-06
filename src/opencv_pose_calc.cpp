#include "opencv_pose_calc.h"
#include <iostream>
#include <string>

std::vector<cv::Point2f> GatherImagePoints(TrackedObject *to);
std::vector<cv::Point3f> GatherObjectPoints(TrackedObject *to);
bool isRotationMatrix(cv::Mat &R);
std::vector<cv::Point3f> rotationMatrixToEulerAngles(cv::Mat &R);

void PoseCalculation(TrackedObject *to, Pose *pose)
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

  pose->Pos[0] = -tvec.at<double>(1,0);
  pose->Pos[1] = tvec.at<double>(0,0);
  pose->Pos[2] = -tvec.at<double>(1,1);

  // std::cout << "tvec: " << tvec << std::endl;
  // std::cout << "Rotation Matrix: " << R << std::endl;

  pose->SE3Mat[0] = R.at<double>(0,0);
  pose->SE3Mat[1] = R.at<double>(0,1);
  pose->SE3Mat[2] = R.at<double>(0,2);
  pose->SE3Mat[3] = pose->Pos[0];
  pose->SE3Mat[4] = R.at<double>(1,0);
  pose->SE3Mat[5] = R.at<double>(1,1);
  pose->SE3Mat[6] = R.at<double>(1,2);
  pose->SE3Mat[7] = pose->Pos[1];
  pose->SE3Mat[8] = R.at<double>(2,0);
  pose->SE3Mat[9] = R.at<double>(2,1);
  pose->SE3Mat[10] = R.at<double>(2,2);
  pose->SE3Mat[11] = pose->Pos[2];
  pose->SE3Mat[12] = 0;
  pose->SE3Mat[13] = 0;
  pose->SE3Mat[14] = 0;
  pose->SE3Mat[15] = 1;

  // for (int i = 0; i < 16; i++)
  // {
  //   std::cout << "SE3Mat[ " << i << "]: " << pose->SE3Mat[i] << std::endl;
  // }
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

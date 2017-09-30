#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>
#include <string.h>
#include <survive_cal.h>
#include <linmath.h>
#include <survive_config.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


struct SurviveContext * ctx;
int quit = 0;
int timestamp = 0;
int timestampprev = 0;

int bufferpts[32*2*3][2];
SurvivePose objPose[2];
SurvivePose lhPose[2];
FLT acc[3];
FLT gyro[3];

char buffermts[32*128*3];
int buffertimeto[32*3][2];

int PoseLoaded = 0;

void my_angle_process( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh)
{
  if (so == so->ctx->objs[0] && so->FromLHPose[0].Pos[0] != 0)
    {
      objPose[0].Pos[0] = so->FromLHPose[0].Pos[0];
      objPose[0].Pos[1] = so->FromLHPose[0].Pos[1];
      objPose[0].Pos[2] = so->FromLHPose[0].Pos[2];

      objPose[0].Rot[0] = so->FromLHPose[0].Rot[0];
      objPose[0].Rot[1] = so->FromLHPose[0].Rot[1];
      objPose[0].Rot[2] = so->FromLHPose[0].Rot[2];
      objPose[0].Rot[3] = so->FromLHPose[0].Rot[3];
		
      timestamp = timecode;
      if (!PoseLoaded)
	{
	  PoseLoaded = 1;
	  printf("Loaded Pose Data\n");
	}
    }

  survive_default_angle_process( so, sensor_id, acode, timecode, length, angle, lh );
}


void my_imu_process( struct SurviveObject * so, int mask, FLT * accelgyro, uint32_t timecode, int id )
{
  survive_default_imu_process( so, mask, accelgyro, timecode, id );

  acc[0] = accelgyro[0];
  acc[1] = accelgyro[1];
  acc[2] = accelgyro[2];

  gyro[0] = accelgyro[3];
  gyro[1] = accelgyro[4];
  gyro[2] = accelgyro[5];

  //if( so->codename[0] == 'H' )
  if( 0 )
    {
      printf( "I %s %d %f %f %f %f %f %f %d\n", so->codename, timecode, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], id );
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vive_pose_broadcaster");

  ctx = survive_init( 0 );

  // this is where the poser is called
  survive_install_angle_fn( ctx, my_angle_process );
  survive_install_imu_fn( ctx,  my_imu_process );

  survive_cal_install( ctx );

  if( !ctx )
    {
      fprintf( stderr, "Fatal. Could not start\n" );
      exit( 1 );
    }


  // setup ROS stuff:
  ros::NodeHandle n;
  tf2_ros::TransformBroadcaster br;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("tracker_pose", 1);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("tracker_imu", 1);

  // Put main code here
  // Full pose (Pos[3] and Quat[4]) will be available in objPose[0]
  while(survive_poll(ctx) == 0 && !quit)
    {
      if (PoseLoaded && (timestamp != timestampprev))
        {
	  geometry_msgs::TransformStamped transformStamped;
	  transformStamped.header.stamp = ros::Time::now();
	  transformStamped.header.frame_id = "lighthouse";
	  transformStamped.child_frame_id = "tracker";
	  transformStamped.transform.translation.x = objPose[0].Pos[0];
	  transformStamped.transform.translation.y = objPose[0].Pos[1];
	  transformStamped.transform.translation.z = -objPose[0].Pos[2];
	  transformStamped.transform.rotation.x = objPose[0].Rot[0];
	  transformStamped.transform.rotation.y = objPose[0].Rot[1];
	  transformStamped.transform.rotation.z = objPose[0].Rot[2];
	  transformStamped.transform.rotation.w = objPose[0].Rot[3];
	  br.sendTransform(transformStamped);

	  // now we can convert to a PoseStamped and pub:
	  geometry_msgs::PoseStamped p;
	  p.header = transformStamped.header;
	  p.pose.position.x = transformStamped.transform.translation.x;
	  p.pose.position.y = transformStamped.transform.translation.y;
	  p.pose.position.z = transformStamped.transform.translation.z;
	  p.pose.orientation.x = transformStamped.transform.rotation.x;
	  p.pose.orientation.y = transformStamped.transform.rotation.y;
	  p.pose.orientation.z = transformStamped.transform.rotation.z;
	  p.pose.orientation.w = transformStamped.transform.rotation.w;
	  pose_pub.publish(p);

	  // publish IMU
	  sensor_msgs::Imu imu;
	  imu.linear_acceleration.x = acc[0];
	  imu.linear_acceleration.y = acc[1];
	  imu.linear_acceleration.z = acc[2];

	  imu.angular_velocity.x = gyro[0];
	  imu.angular_velocity.y = gyro[1];
	  imu.angular_velocity.z = gyro[2];
	  imu_pub.publish(imu);
	  
	  ROS_DEBUG_THROTTLE(0.5, "time: %d -- time_previous: %d : (x,y,z) = (%f, %f, %f)", timestampprev, timestamp, objPose[0].Pos[0], objPose[0].Pos[1], objPose[0].Pos[2]);
        }
      timestampprev = timestamp;
    }

  survive_close( ctx );

  printf("Exiting main.\n");
  return 0;
}

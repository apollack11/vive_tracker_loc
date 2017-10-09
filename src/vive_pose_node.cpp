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
SurvivePose prevPose[2];
SurvivePose lhPose[2];

SurviveImu imuData;

char buffermts[32*128*3];
int buffertimeto[32*3][2];

int PoseLoaded = 0;
int ImuLoaded = 0;

void my_angle_process( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh)
{
	survive_default_angle_process( so, sensor_id, acode, timecode, length, angle, lh );

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
			prevPose[0].Pos[0] = objPose[0].Pos[0];
			prevPose[0].Pos[1] = objPose[0].Pos[1];
			prevPose[0].Pos[2] = objPose[0].Pos[2];
			prevPose[0].Rot[0] = objPose[0].Rot[0];
			prevPose[0].Rot[1] = objPose[0].Rot[1];
			prevPose[0].Rot[2] = objPose[0].Rot[2];
			prevPose[0].Rot[3] = objPose[0].Rot[3];
			PoseLoaded = 1;
			printf("Loaded Pose Data\n");
		}
    }
}

void my_imu_process( struct SurviveObject * so, int mask, FLT * accelgyro, uint32_t timecode, int id)
{
	survive_default_imu_process( so, mask, accelgyro, timecode, id );

	if (so == so->ctx->objs[0] && so->ImuData.Accel[0] != 0)
    {
		imuData.Accel[0] = so->ImuData.Accel[0];
		imuData.Accel[1] = so->ImuData.Accel[1];
		imuData.Accel[2] = so->ImuData.Accel[2];
		imuData.Gyro[0] = so->ImuData.Gyro[0];
		imuData.Gyro[1] = so->ImuData.Gyro[1];
		imuData.Gyro[2] = so->ImuData.Gyro[2];

		if (!ImuLoaded)
		{
			ImuLoaded = 1;
			printf("Loaded IMU Data\n");
		}
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vive_pose_broadcaster");
	ctx = survive_init( 0 );

	// this is where the poser is called
	survive_install_angle_fn( ctx, my_angle_process );
	survive_install_imu_fn( ctx, my_imu_process );

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
		if (PoseLoaded && ImuLoaded && (timestamp != timestampprev))
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
			geometry_msgs::Vector3 lin_accel;
			lin_accel.x = imuData.Accel[0];
			lin_accel.y = imuData.Accel[1];
			lin_accel.z = imuData.Accel[2];
			geometry_msgs::Vector3 ang_vel;
			ang_vel.x = imuData.Gyro[0];
			ang_vel.y = imuData.Gyro[1];
			ang_vel.z = imuData.Gyro[2];
			sensor_msgs::Imu imu_msg;
			imu_msg.linear_acceleration = lin_accel;
			imu_msg.angular_velocity = ang_vel;
			imu_pub.publish(imu_msg);

			ROS_DEBUG_THROTTLE(0.5, "time: %d -- time_previous: %d : (x, y, z) = (%f, %f, %f)  |  (x, y, z, w) = (%f,%f,%f,%f)",
							   timestampprev, timestamp, p.pose.position.x, p.pose.position.y, p.pose.position.z,
							   p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
			ROS_DEBUG_THROTTLE(0.5, "time: %d -- time_previous: %d : (xdd, ydd, zdd) = (%f, %f, %f)  |  (wx, wy, wz) = (%f, %f, %f)\n",
							   timestampprev, timestamp, lin_accel.x, lin_accel.y, lin_accel.z, ang_vel.x, ang_vel.y, ang_vel.z);
		}
		timestampprev = timestamp;
    }

	survive_close( ctx );

	printf("Exiting main.\n");
	return 0;
}

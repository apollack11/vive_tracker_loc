#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>
#include <string.h>
#include <survive_cal.h>
#include <linmath.h>
#include <survive_config.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

struct SurviveContext * ctx;
int quit = 0;
int timestamp = 0;
int timestampprev = 0;

int bufferpts[32*2*3][2];
SurvivePose objPose[2];
SurvivePose prevPose[2];
SurvivePose lhPose[2];

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

  survive_default_angle_process( so, sensor_id, acode, timecode, length, angle, lh );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vive_pose_broadcaster");

  ctx = survive_init( 0 );

  // this is where the poser is called
  survive_install_angle_fn( ctx, my_angle_process );

  survive_cal_install( ctx );

  if( !ctx )
    {
      fprintf( stderr, "Fatal. Could not start\n" );
      exit( 1 );
    }

  // Put main code here
  // Full pose (Pos[3] and Quat[4]) will be available in objPose[0]
  while(survive_poll(ctx) == 0 && !quit)
    {
      if (PoseLoaded && (timestamp != timestampprev))
        {
	  static tf::TransformBroadcaster br;
	  tf::Transform transform;
	  if (abs(prevPose[0].Pos[0] - objPose[0].Pos[0]) > 0.2 || abs(prevPose[0].Pos[0] - objPose[0].Pos[0]) > 0.2 || abs(prevPose[0].Pos[0] - objPose[0].Pos[0]) > 0.2) 
	  {
	    transform.setOrigin(tf::Vector3(prevPose[0].Pos[0], prevPose[0].Pos[1], prevPose[0].Pos[2]));
	    tf::Quaternion q = tf::Quaternion(prevPose[0].Rot[0], prevPose[0].Rot[1], prevPose[0].Rot[2], prevPose[0].Rot[3]);
	    transform.setRotation(q);
	  }
	  else 
	  {
	    transform.setOrigin(tf::Vector3(objPose[0].Pos[0], objPose[0].Pos[1], objPose[0].Pos[2]));
	    tf::Quaternion q = tf::Quaternion(objPose[0].Rot[0], objPose[0].Rot[1], objPose[0].Rot[2], objPose[0].Rot[3]);
	    transform.setRotation(q);
	  }
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "lighthouse", "tracker"));
	  // printf("%d -- %d : (%f, %f, %f)\n", timestampprev, timestamp, objPose[0].Pos[0], objPose[0].Pos[1], objPose[0].Pos[2]);
        }
      timestampprev = timestamp;
      prevPose[0].Pos[0] = objPose[0].Pos[0];
      prevPose[0].Pos[1] = objPose[0].Pos[1];
      prevPose[0].Pos[2] = objPose[0].Pos[2];
      prevPose[0].Rot[0] = objPose[0].Rot[0];
      prevPose[0].Rot[1] = objPose[0].Rot[1];
      prevPose[0].Rot[2] = objPose[0].Rot[2];
      prevPose[0].Rot[3] = objPose[0].Rot[3];
    }

  survive_close( ctx );

  printf("Exiting main.\n");
  return 0;
}

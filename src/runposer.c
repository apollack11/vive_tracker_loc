// recorder mod with GUI showing light positions.

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>
#include <string.h>
#include <os_generic.h>
#include "src/survive_cal.h"
#include <linmath.h>

#include "src/survive_config.h"

struct SurviveContext * ctx;
int  quit = 0;

int bufferpts[32*2*3][2];
SurvivePose objPose[2];
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
		
		if (!PoseLoaded)
		{
		  PoseLoaded = 1;
		  printf("Loaded Pose Data\n");
		}
	}
	survive_default_angle_process( so, sensor_id, acode, timecode, length, angle, lh );
}

int main()
{
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
	        if (PoseLoaded)
	        {
	                printf("Current position: (%f, %f, %f)\n", objPose[0].Pos[0], objPose[0].Pos[1], objPose[0].Pos[2]);
	        }
       	}

	survive_close( ctx );

	printf("Exiting main.\n");
	return 0;
}



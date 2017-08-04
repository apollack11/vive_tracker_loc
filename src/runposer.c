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

void my_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length, uint32_t lh)
{
//	if( timeinsweep < 0 ) return;
	survive_default_light_process( so, sensor_id, acode, timeinsweep, timecode, length, lh);
	if( sensor_id < 0 ) return;
	if( acode < 0 ) return;
//return;
	int jumpoffset = sensor_id;
	if( strcmp( so->codename, "WM0" ) == 0 || strcmp( so->codename, "WW0" ) == 0) jumpoffset += 32;
	else if( strcmp( so->codename, "WM1" ) == 0 ) jumpoffset += 64;

	// If this is the first tracked object and the pose has been set to something...
	if (so == so->ctx->objs[0] && so->FromLHPose[0].Pos[0] != 0)
	{
		objPose[0].Pos[0] = so->FromLHPose[0].Pos[0];
		objPose[0].Pos[1] = so->FromLHPose[0].Pos[1];
		objPose[0].Pos[2] = so->FromLHPose[0].Pos[2];

		objPose[0].Rot[0] = so->FromLHPose[0].Rot[0];
		objPose[0].Rot[1] = so->FromLHPose[0].Rot[1];
		objPose[0].Rot[2] = so->FromLHPose[0].Rot[2];
		objPose[0].Rot[3] = so->FromLHPose[0].Rot[3];

		lhPose[0].Pos[0] = so->ctx->bsd[0].Pose.Pos[0];
		lhPose[0].Pos[1] = so->ctx->bsd[0].Pose.Pos[1];
		lhPose[0].Pos[2] = so->ctx->bsd[0].Pose.Pos[2];

		lhPose[0].Rot[0] = so->ctx->bsd[0].Pose.Rot[0];
		lhPose[0].Rot[1] = so->ctx->bsd[0].Pose.Rot[1];
		lhPose[0].Rot[2] = so->ctx->bsd[0].Pose.Rot[2];
		lhPose[0].Rot[3] = so->ctx->bsd[0].Pose.Rot[3];

		//quatgetreciprocal(lhPose[0].Rot, lhPose[0].Rot);
	}
	if (so == so->ctx->objs[0] && so->FromLHPose[1].Pos[0] != 0)
	{
		objPose[1].Pos[0] = so->FromLHPose[1].Pos[0];
		objPose[1].Pos[1] = so->FromLHPose[1].Pos[1];
		objPose[1].Pos[2] = so->FromLHPose[1].Pos[2];

		objPose[1].Rot[0] = so->FromLHPose[1].Rot[0];
		objPose[1].Rot[1] = so->FromLHPose[1].Rot[1];
		objPose[1].Rot[2] = so->FromLHPose[1].Rot[2];
		objPose[1].Rot[3] = so->FromLHPose[1].Rot[3];

		lhPose[1].Pos[0] = so->ctx->bsd[1].Pose.Pos[0];
		lhPose[1].Pos[1] = so->ctx->bsd[1].Pose.Pos[1];
		lhPose[1].Pos[2] = so->ctx->bsd[1].Pose.Pos[2];

		lhPose[1].Rot[0] = so->ctx->bsd[1].Pose.Rot[0];
		lhPose[1].Rot[1] = so->ctx->bsd[1].Pose.Rot[1];
		lhPose[1].Rot[2] = so->ctx->bsd[1].Pose.Rot[2];
		lhPose[1].Rot[3] = so->ctx->bsd[1].Pose.Rot[3];

		//quatgetreciprocal(lhPose[1].Rot, lhPose[1].Rot);

	}

	if( acode % 2 == 0 && lh == 0) //data = 0
	{
		bufferpts[jumpoffset*2+0][0] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][0] = 0;
	}
	if(  acode % 2 == 1 && lh == 0 ) //data = 1
	{
		bufferpts[jumpoffset*2+1][0] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][0] = 0;
	}


	if( acode % 2 == 0 && lh == 1 ) //data = 0
	{
		bufferpts[jumpoffset*2+0][1] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][1] = 0;
	}
	if( acode % 2 == 1 && lh == 1 ) //data = 1
	{
		bufferpts[jumpoffset*2+1][1] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][1] = 0;
	}
}

void my_imu_process( struct SurviveObject * so, int mask, FLT * accelgyro, uint32_t timecode, int id )
{
	survive_default_imu_process( so, mask, accelgyro, timecode, id );

	//if( so->codename[0] == 'H' )
	if( 0 )
	{
		printf( "I %s %d %f %f %f %f %f %f %d\n", so->codename, timecode, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], id );
	}
}


void my_angle_process( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh)
{
	survive_default_angle_process( so, sensor_id, acode, timecode, length, angle, lh );
}


char* sensor_name[32];

int main()
{
	ctx = survive_init( 0 );

	uint8_t i =0;
	for (i=0;i<32;++i) {
	        sensor_name[i] = (char*)malloc(3);
		sprintf(sensor_name[i],"%d",i);
	}

	survive_install_light_fn( ctx,  my_light_process );
	survive_install_imu_fn( ctx,  my_imu_process );
	survive_install_angle_fn( ctx, my_angle_process );

	survive_cal_install( ctx );

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		exit( 1 );
	}

	while(survive_poll(ctx) == 0 && !quit)
	{
		//Do stuff.
	}

	survive_close( ctx );

	printf( "Returned\n" );
	return 0;
}



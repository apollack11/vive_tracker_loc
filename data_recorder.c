//Data recorder mod with GUI showing light positions.

#ifdef __linux__
#include <unistd.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>
#include <string.h>
#include <os_generic.h>
#include <CNFGFunctions.h>

struct SurviveContext * ctx;

void HandleKey( int keycode, int bDown )
{
	if( !bDown ) return;

	if( keycode == 'O' || keycode == 'o' )
	{
		survive_send_magic(ctx,1,0,0);
	}
	if( keycode == 'F' || keycode == 'f' )
	{
		survive_send_magic(ctx,0,0,0);
	}
}

void HandleButton( int x, int y, int button, int bDown )
{
}

void HandleMotion( int x, int y, int mask )
{
}

void HandleDestroy()
{
}

int bufferpts[32*2*3];
char buffermts[32*128*3];
int buffertimeto[32*3];

void my_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length, uint32_t lh)
{
	survive_default_light_process( so, sensor_id, acode, timeinsweep, timecode, length, lh);

	if( acode == -1 ) return;
//return;
	int jumpoffset = sensor_id;
	if( strcmp( so->codename, "WM0" ) == 0 ) jumpoffset += 32;
	else if( strcmp( so->codename, "WM1" ) == 0 ) jumpoffset += 64;


	if( acode == 0 || acode == 2 ) //data = 0
	{
		printf( "L X %s %d %d %d %d %d\n", so->codename, timecode, sensor_id, acode, timeinsweep, length );
		bufferpts[jumpoffset*2+0] = (timeinsweep-100000)/500;
		buffertimeto[jumpoffset] = 0;
	}
	if( acode == 1 || acode == 3 ) //data = 1
	{
		printf( "L Y %s %d %d %d %d %d\n", so->codename, timecode, sensor_id, acode, timeinsweep, length );
		bufferpts[jumpoffset*2+1] = (timeinsweep-100000)/500;
		buffertimeto[jumpoffset] = 0;
	}


	if( acode == 4 || acode == 6 ) //data = 0
	{
		printf( "R X %s %d %d %d %d %d\n", so->codename, timecode, sensor_id, acode, timeinsweep, length );
		bufferpts[jumpoffset*2+0] = (timeinsweep-100000)/500;
		buffertimeto[jumpoffset] = 0;
	}
	if( acode == 5 || acode == 7 ) //data = 1
	{
		printf( "R Y %s %d %d %d %d %d\n", so->codename, timecode, sensor_id, acode, timeinsweep, length );
		bufferpts[jumpoffset*2+1] = (timeinsweep-100000)/500;
		buffertimeto[jumpoffset] = 0;
	}

}

void my_imu_process( struct SurviveObject * so, int mask, FLT * accelgyro, uint32_t timecode, int id )
{
	survive_default_imu_process( so, mask, accelgyro, timecode, id );

//return;
	//if( so->codename[0] == 'H' )
	if( 1 )
	{
		printf( "I %s %d %f %f %f %f %f %f %d\n", so->codename, timecode, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], id );
	}
}




void * GuiThread( void * v )
{
	CNFGBGColor = 0x000000;
	CNFGDialogColor = 0x444444;
	CNFGSetup( "Survive GUI Debug", 640, 480 );

	short screenx, screeny;
	while(1)
	{
		CNFGHandleInput();
		CNFGClearFrame();
		CNFGColor( 0xFFFFFF );
		CNFGGetDimensions( &screenx, &screeny );

		int i;
		for( i = 0; i < 32*3; i++ )
		{
			if( buffertimeto[i] < 50 )
			{
				uint32_t color = i * 3231349;
				uint8_t r = color & 0xff;
				uint8_t g = (color>>8) & 0xff;
				uint8_t b = (color>>16) & 0xff;
				r = (r * (5-buffertimeto[i])) / 5 ;
				g = (g * (5-buffertimeto[i])) / 5 ;
				b = (b * (5-buffertimeto[i])) / 5 ;
				CNFGColor( (b<<16) | (g<<8) | r );
				CNFGTackRectangle( bufferpts[i*2+0], bufferpts[i*2+1], bufferpts[i*2+0] + 5, bufferpts[i*2+1] + 5 );
				CNFGPenX = bufferpts[i*2+0]; CNFGPenY = bufferpts[i*2+1];
				CNFGDrawText( buffermts, 2 );			
				buffertimeto[i]++;
			}
		}

		CNFGSwapBuffers();
		OGUSleep( 10000 );
	}
}

int SurviveThreadLoaded=0;

void *SurviveThread(void *junk)
{
	ctx = survive_init( 0 );

	survive_install_light_fn( ctx,  my_light_process );
	survive_install_imu_fn( ctx,  my_imu_process );

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		exit(1);
	}

	SurviveThreadLoaded = 1;

	while(survive_poll(ctx) == 0)
	{
        printf("Do stuff.\n");
		//Do stuff.
	}

    return 0;
}

int main()
{
    // Create the libsurvive thread
    OGCreateThread(SurviveThread, 0);
    
	// Wait for the survive thread to load
	while (!SurviveThreadLoaded) { OGUSleep(100); }
	
    // Run the Gui in the main thread
    GuiThread(0);
}


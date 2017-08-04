//<>< (C) 2016 C. N. Lohr, MOSTLY Under MIT/x11 License.
//

#include "survive_internal.h"
#include <stdint.h>
#include <string.h>
#include <math.h> /* for sqrt */

#define USE_TURVEYBIGUATOR

#ifdef USE_TURVEYBIGUATOR

static const float tau_table[33] = { 0, 0, 0, 1.151140982, 1.425, 1.5712213707, 1.656266074, 1.7110275587, 1.7490784054,
	1.7770229476, 1.798410005, 1.8153056661, 1.8289916275, 1.8403044103, 1.8498129961, 1.8579178211,
	1.864908883, 1.8710013691, 1.8763583296, 1.881105575, 1.885341741, 1.8891452542, 1.8925792599,
	1.8956951735, 1.8985352854, 1.9011347009, 1.9035228046, 1.9057243816, 1.9077604832, 1.9096491058,
	1.9114057255, 1.9130437248, 1.914574735
};

typedef struct
{
	unsigned int sweep_time[SENSORS_PER_OBJECT];
	uint16_t sweep_len[SENSORS_PER_OBJECT]; //might want to align this to cache lines, will be hot for frequent access
} lightcaps_sweep_data;

typedef struct
{
	int recent_sync_time;
	int activeLighthouse;
	int activeSweepStartTime;
	int activeAcode;

//	int lh_pulse_len[NUM_LIGHTHOUSES];
	int lh_start_time[NUM_LIGHTHOUSES];
	int lh_max_pulse_length[NUM_LIGHTHOUSES];
	int8_t lh_acode[NUM_LIGHTHOUSES];
	int current_lh; // used knowing which sync pulse we're looking at.

} lightcap2_per_sweep_data;

typedef struct
{
	double acode_offset;
} lightcap2_global_data;

typedef struct
{
	lightcaps_sweep_data sweep;
	lightcap2_per_sweep_data per_sweep;
	lightcap2_global_data global;
} lightcap2_data;


//static lightcap2_global_data lcgd = { 0 };

int handle_lightcap2_getAcodeFromSyncPulse(SurviveObject * so, int pulseLen)
{
	double oldOffset = ((lightcap2_data*)so->disambiguator_data)->global.acode_offset;

	int modifiedPulseLen = pulseLen - (int)oldOffset;

	double newOffset = (((pulseLen) + 250) % 500) - 250;

	((lightcap2_data*)so->disambiguator_data)->global.acode_offset = oldOffset * 0.9 + newOffset * 0.1;

//fprintf(stderr, "    %f\n", oldOffset);
#define ACODE_OFFSET 0
	if (pulseLen < 3250 - ACODE_OFFSET) return 0;
	if (pulseLen < 3750 - ACODE_OFFSET) return 1;
	if (pulseLen < 4250 - ACODE_OFFSET) return 2;
	if (pulseLen < 4750 - ACODE_OFFSET) return 3;
	if (pulseLen < 5250 - ACODE_OFFSET) return 4;
	if (pulseLen < 5750 - ACODE_OFFSET) return 5;
	if (pulseLen < 6250 - ACODE_OFFSET) return 6;
	return 7;
}

uint8_t remove_outliers(SurviveObject *so) {
	return 0; // disabling this for now because it seems remove almost all the points for wired watchman and wired tracker.
	lightcap2_data *lcd = so->disambiguator_data;

	uint32_t sum = 0;
	uint8_t non_zero_count = 0;
	uint32_t mean = 0;

	uint16_t* min = NULL;
	uint16_t* max = NULL;
	uint8_t found_first = 0;

	//future: https://gcc.gnu.org/projects/tree-ssa/vectorization.html#vectorizab

	for (uint8_t i = 0; i < SENSORS_PER_OBJECT; i++)
	{
		sum += lcd->sweep.sweep_len[i];
		if (lcd->sweep.sweep_len[i] > 0) ++non_zero_count;
	}

	if (non_zero_count==0) return 0;

	mean = sum/non_zero_count;

	float standard_deviation = 0.0f;
	sum = 0;
	for (uint8_t i = 0; i < SENSORS_PER_OBJECT; i++)
	{
		uint16_t len = lcd->sweep.sweep_len[i];
		if (len > 0) {
			sum += (len - mean)*(len - mean);

			if (found_first==0) {
				max = min = lcd->sweep.sweep_len + i;
				found_first=1;
			} else {
				if(lcd->sweep.sweep_len[i] < *min) min=lcd->sweep.sweep_len + i;
				if(lcd->sweep.sweep_len[i] > *max) max=lcd->sweep.sweep_len + i;
			}
		}
	}
	standard_deviation = sqrtf( ((float)sum)/((float)non_zero_count) );

//	printf("%f\n", standard_deviation);

	float tau_test = standard_deviation;

	if (non_zero_count > 2) tau_test = standard_deviation*tau_table[non_zero_count];

//	uint8_t removed_outliers = 0;

	uint32_t d1 = abs(*min - mean);
	uint32_t d2 = abs(*max - mean);

	if (d1>d2) {
		if (d1 > tau_test) {
			*min = 0;
			return 1;
		}
	}
	else if (d2>tau_test) {
		*max = 0;
		return 1;
	}

	return 0;
/*
	for (uint8_t i = 0; i < SENSORS_PER_OBJECT; i++)
	{
		uint16_t len = lcd->sweep.sweep_len[i];
		if (len == 0) continue;

		if ( abs(len-mean) > tau_test )
		{
//			fprintf(stderr, "removing %d\n", len);
			lcd->sweep.sweep_len[i] = 0;
			removed_outliers = 1;
		}
	}
*/
//	return removed_outliers;
}

void handle_lightcap2_process_sweep_data(SurviveObject *so)
{
	lightcap2_data *lcd = so->disambiguator_data;

	while(remove_outliers(so));

	// look at all of the sensors we found, and process the ones that were hit.
	// TODO: find the sensor(s) with the longest pulse length, and assume 
	// those are the "highest quality".  Then, reject any pulses that are sufficiently
	// different from those values, assuming that they are reflections.
	{
		unsigned int longest_pulse = 0;
		unsigned int timestamp_of_longest_pulse = 0;
		for (int i = 0; i < SENSORS_PER_OBJECT; i++)
		{
			if (lcd->sweep.sweep_len[i] > longest_pulse)
			{
				longest_pulse = lcd->sweep.sweep_len[i];
				timestamp_of_longest_pulse = lcd->sweep.sweep_time[i];
			}
		}

		int allZero = 1;
		for (int q=0; q< 32; q++)
			if (lcd->sweep.sweep_len[q] != 0)
				allZero=0;
		//if (!allZero)
		//	printf("a[%d]l[%d] ", lcd->per_sweep.activeAcode & 5,  lcd->per_sweep.activeLighthouse);
		for (int i = 0; i < SENSORS_PER_OBJECT; i++)
		{

			{
				static int counts[SENSORS_PER_OBJECT][2] = {0};

//				if (lcd->per_sweep.activeLighthouse == 0 && !allZero)
				if (lcd->per_sweep.activeLighthouse > -1 && !allZero)
				{
					if (lcd->sweep.sweep_len[i] != 0)
					{
						//printf("%d  ", i);
						//counts[i][lcd->per_sweep.activeAcode & 1] ++;
					}
					else
					{
						counts[i][lcd->per_sweep.activeAcode & 1] =0;
					}

					//if (counts[i][0] > 10 && counts[i][1] > 10)
					//{
						//printf("%d(%d,%d), ", i, counts[i][0], counts[i][1]);
					//}
				}
			}


			if (lcd->sweep.sweep_len[i] != 0) // if the sensor was hit, process it
			{
//printf("%4d\n", lcd->sweep.sweep_len[i]);
				int offset_from = lcd->sweep.sweep_time[i] - lcd->per_sweep.activeSweepStartTime + lcd->sweep.sweep_len[i] / 2;

//				if (offset_from < 380000 && offset_from > 70000)
				{
					//if (longest_pulse *10 / 8 < lcd->sweep.sweep_len[i]) 
					{
						so->ctx->lightproc(so, i, lcd->per_sweep.activeAcode, offset_from, lcd->sweep.sweep_time[i], lcd->sweep.sweep_len[i], lcd->per_sweep.activeLighthouse);
					}
				}
			}
		}
		//if (!allZero)
		//	printf("   ..:..\n");

//		if (!allZero) printf("\n");
	}

	// clear out sweep data (could probably limit this to only after a "first" sync.  
	// this is slightly more robust, so doing it here for now.
	memset(&(((lightcap2_data*)so->disambiguator_data)->sweep), 0, sizeof(lightcaps_sweep_data));
}
void handle_lightcap2_sync(SurviveObject * so, LightcapElement * le )
{
	//fprintf(stderr, "%6.6d %4.4d \n", le->timestamp - so->recent_sync_time, le->length);
	lightcap2_data *lcd = so->disambiguator_data;

	//static unsigned int recent_sync_time = 0;
	//static unsigned int recent_sync_count = -1;
	//static unsigned int activeSweepStartTime;

	int acode = handle_lightcap2_getAcodeFromSyncPulse(so, le->length); //acode for this sensor reading

	// Process any sweep data we have
	handle_lightcap2_process_sweep_data(so);

	int time_since_last_sync = (le->timestamp - lcd->per_sweep.recent_sync_time);


//	fprintf(stderr, "            %2d %8d %d\n", le->sensor_id, time_since_last_sync, le->length);
	// need to store up sync pulses, so we can take the earliest starting time for all sensors.
	if (time_since_last_sync < 2400)
	{
		lcd->per_sweep.recent_sync_time = le->timestamp;
		// it's the same sync pulse;
//		so->sync_set_number = 1;
		so->recent_sync_time = le->timestamp;

//		lcd->per_sweep.lh_pulse_len[lcd->per_sweep.current_lh] = le->length;
//		lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = le->timestamp;

		if (le->length > lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh]) {
			lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh] = le->length;
			lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = le->timestamp;
			lcd->per_sweep.lh_acode[lcd->per_sweep.current_lh] = acode;
		}

/*
		//this stuff should probably be happening on the sweep so that we can filter out erroneous a codes
		if (!(acode >> 2 & 1)) // if the skip bit is not set
		{
			lcd->per_sweep.activeLighthouse = lcd->per_sweep.current_lh;
			lcd->per_sweep.activeSweepStartTime = le->timestamp;
			lcd->per_sweep.activeAcode = acode;
		}
		else
		{
			//this causes the master lighthouse to be ignored from the HMD
			lcd->per_sweep.activeLighthouse = -1;
			lcd->per_sweep.activeSweepStartTime = 0;
			lcd->per_sweep.activeAcode = 0;
		}
		*/
	}
	else if (time_since_last_sync < 24000)
	{
		lcd->per_sweep.activeLighthouse = -1;

		lcd->per_sweep.recent_sync_time = le->timestamp;
		// I do believe we are lighthouse B		
		lcd->per_sweep.current_lh = 1;
//		lcd->per_sweep.lh_pulse_len[lcd->per_sweep.current_lh] = le->length;
		lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = le->timestamp;
		lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh] = le->length;
		lcd->per_sweep.lh_acode[lcd->per_sweep.current_lh] = acode;

/*
		if (!(acode >> 2 & 1)) // if the skip bit is not set
		{
			if (lcd->per_sweep.activeLighthouse != -1)
			{
				static int pulseWarningCount=0;

				if (pulseWarningCount < 5)
				{
					pulseWarningCount++;
					// hmm, it appears we got two non-skip pulses at the same time.  That should never happen
					fprintf(stderr, "WARNING: Two non-skip pulses received on the same cycle!\n");
				}
			}

			lcd->per_sweep.activeLighthouse = 1;
			lcd->per_sweep.activeSweepStartTime = le->timestamp;
			lcd->per_sweep.activeAcode = acode;
		}
		*/

	}
	else if (time_since_last_sync > 370000)
	{
		// XXX CAUTION: if we lose sight of a lighthouse then, the remaining lighthouse will default to master
		//this should probably be fixed. Maybe some kind of timing based guess at which lighthouse.

		// looks like this is the first sync pulse.  Cool!

		// first, send out the sync pulse data for the last round (for OOTX decoding
		{
			if (lcd->per_sweep.lh_max_pulse_length[0] != 0)
			{
				so->ctx->lightproc(
					so,
					-1,
					handle_lightcap2_getAcodeFromSyncPulse(so, lcd->per_sweep.lh_max_pulse_length[0]),
					lcd->per_sweep.lh_max_pulse_length[0],
					lcd->per_sweep.lh_start_time[0],
					0,
					0);
			}
			if (lcd->per_sweep.lh_max_pulse_length[1] != 0)
			{
				so->ctx->lightproc(
					so,
					-2,
					handle_lightcap2_getAcodeFromSyncPulse(so, lcd->per_sweep.lh_max_pulse_length[1]),
					lcd->per_sweep.lh_max_pulse_length[1],
					lcd->per_sweep.lh_start_time[1],
					0,
					1);
			}
		}

		//fprintf(stderr, "************************************ Reinitializing Disambiguator!!!\n");
		// initialize here.
		memset(&lcd->per_sweep, 0, sizeof(lcd->per_sweep));
		lcd->per_sweep.activeLighthouse = -1; 

		for (uint8_t i=0; i < NUM_LIGHTHOUSES;++i) {
			lcd->per_sweep.lh_acode[i] = -1;
		}

		lcd->per_sweep.recent_sync_time = le->timestamp;
		// I do believe we are lighthouse A		
		lcd->per_sweep.current_lh = 0;
//		lcd->per_sweep.lh_pulse_len[lcd->per_sweep.current_lh] = le->length;
		lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = le->timestamp;
		lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh] = le->length;
		lcd->per_sweep.lh_acode[lcd->per_sweep.current_lh] = acode;

//		int acode = handle_lightcap2_getAcodeFromSyncPulse(so, le->length);

/*
		if (!(acode >> 2 & 1)) // if the skip bit is not set
		{
			lcd->per_sweep.activeLighthouse = 0;
			lcd->per_sweep.activeSweepStartTime = le->timestamp;
			lcd->per_sweep.activeAcode = acode;
		}
		*/
	}

//	printf("%d %d\n", acode, lcd->per_sweep.activeLighthouse );
}

void handle_lightcap2_sweep(SurviveObject * so, LightcapElement * le )
{
	lightcap2_data *lcd = so->disambiguator_data;

	// If we see multiple "hits" on the sweep for a given sensor,
	// assume that the longest (i.e. strongest signal) is most likely 
	// the non-reflected signal. 

	//if (le->length < 80)
	//{
	//	// this is a low-quality read.  Better to throw it out than to use it.
	//	//fprintf(stderr, "%2d %d\n", le->sensor_id, le->length);
	//	return;
	//}
	//fprintf(stderr, "%2d %d\n", le->sensor_id, le->length);
	//fprintf(stderr, ".");

	lcd->per_sweep.activeLighthouse = -1;
	lcd->per_sweep.activeSweepStartTime = 0;
	lcd->per_sweep.activeAcode = 0;

	for (uint8_t i=0; i < NUM_LIGHTHOUSES;++i) {
		int acode = lcd->per_sweep.lh_acode[i];
		if ( (acode>=0) && !(acode >> 2 & 1)) {
			lcd->per_sweep.activeLighthouse = i;
			lcd->per_sweep.activeSweepStartTime = lcd->per_sweep.lh_start_time[i];
			lcd->per_sweep.activeAcode = acode;
		}
	}

	if (lcd->per_sweep.activeLighthouse < 0) {
		fprintf(stderr, "WARNING: No active lighthouse!\n");
		fprintf(stderr, "            %2d %8d %d %d\n", le->sensor_id, le->length,lcd->per_sweep.lh_acode[0],lcd->per_sweep.lh_acode[1]);
		return;
	}

	if (lcd->sweep.sweep_len[le->sensor_id] < le->length)
	{
		lcd->sweep.sweep_len[le->sensor_id] = le->length;
		lcd->sweep.sweep_time[le->sensor_id] = le->timestamp;
	}
}

void handle_lightcap2( SurviveObject * so, LightcapElement * le )
{
	SurviveContext * ctx = so->ctx;

	if (so->disambiguator_data == NULL)
	{
		fprintf(stderr, "Initializing Disambiguator Data\n");
		so->disambiguator_data = malloc(sizeof(lightcap2_data));
		memset(so->disambiguator_data, 0, sizeof(lightcap2_data));
	}

	if( le->sensor_id > SENSORS_PER_OBJECT )
	{
		return;
	}

	if (le->length > 6750)
	{
		// Should never get a reading so high.  Odd.
		return;
	}
//	if (le->length >= 2750)
	if (le->length >= 2500)
	{
		// Looks like a sync pulse, process it!
		handle_lightcap2_sync(so, le);
		return;
	}

	// must be a sweep pulse, process it!
	handle_lightcap2_sweep(so, le);

}


#endif


int32_t decode_acode(uint32_t length, int32_t main_divisor) {
	//+50 adds a small offset and seems to help always get it right. 
	//Check the +50 in the future to see how well this works on a variety of hardware.
	if( !main_divisor ) return -1;
	int32_t acode = (length+main_divisor+50)/(main_divisor*2);
	if( acode & 1 ) return -1;

	return (acode>>1) - 6;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
///////The charles disambiguator.  Don't use this, mostly here for debugging.///////////////////////////////////////////////////////	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	


void HandleOOTX( SurviveContext * ctx, SurviveObject * so )
{
	int32_t main_divisor = so->timebase_hz / 384000; //125 @ 48 MHz.

	int32_t acode_array[2] =
		{
			decode_acode(so->last_sync_length[0],main_divisor),
			decode_acode(so->last_sync_length[1],main_divisor)
		};


	int32_t delta1 = so->last_sync_time[0] - so->recent_sync_time;
	int32_t delta2 = so->last_sync_time[1] - so->last_sync_time[0];

	//printf( "%p %p %d %d %d  %p\n", ctx, so, so->last_sync_time[0], acode_array, so->last_sync_length[0], ctx->lightproc );
	if( acode_array[0] >= 0 ) ctx->lightproc( so, -1, acode_array[0], delta1, so->last_sync_time[0], so->last_sync_length[0], 0 );
	if( acode_array[1] >= 0 ) ctx->lightproc( so, -2, acode_array[1], delta2, so->last_sync_time[1], so->last_sync_length[1], 1 );

	so->recent_sync_time = so->last_sync_time[1];

	so->did_handle_ootx = 1;
}


//This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
void handle_lightcap( SurviveObject * so, LightcapElement * le )
{
	SurviveContext * ctx = so->ctx;

#ifdef USE_TURVEYBIGUATOR
	handle_lightcap2(so,le);
	return;

#else
	//printf( "LE%3d%6d%12d\n", le->sensor_id, le->length, le->timestamp );

	//int32_t deltat = (uint32_t)le->timestamp - (uint32_t)so->last_master_time;

	if( le->sensor_id > SENSORS_PER_OBJECT )
	{
		return;
	}

#if 0
	if( so->codename[0] == 'H' )
	{
		static int lt;
		static int last;
		if( le->length > 1000 )
		{
			int dl = le->timestamp - lt;
			lt = le->timestamp;
			if( dl > 10000 || dl < -10000 )
				printf( "+++%s %3d %5d %9d  ", so->codename, le->sensor_id, le->length, dl );
			if( dl > 100000 ) printf(" \n" );
		}
		last=le->length;
	}
#endif

	so->tsl = le->timestamp;
	if( le->length < 20 ) return;  ///Assuming 20 is an okay value for here.

	//The sync pulse finder is taking Charles's old disambiguator code and mixing it with a more linear
	//version of Julian Picht's disambiguator, available in 488c5e9.  Removed afterwards into this
	//unified driver.
	int ssn = so->sync_set_number; //lighthouse number
	if( ssn < 0 ) ssn = 0;
#ifdef DEBUG
	if( ssn >= NUM_LIGHTHOUSES ) { SV_INFO( "ALGORITHMIC WARNING: ssn exceeds NUM_LIGHTHOUSES" ); }
#endif
	int last_sync_time  =  so->last_sync_time  [ssn];
	int last_sync_length = so->last_sync_length[ssn];
	int32_t delta = le->timestamp - last_sync_time;  //Handle time wrapping (be sure to be int32)

	if( delta < -so->pulsedist_max_ticks || delta > so->pulsedist_max_ticks )
	{
		//Reset pulse, etc.
		so->sync_set_number = -1;
		delta = so->pulsedist_max_ticks;
//		return; //if we don't know what lighthouse this is we don't care to do much else
	}


	if( le->length > so->pulselength_min_sync ) //Pulse longer indicates a sync pulse.
	{
		int is_new_pulse = delta > so->pulselength_min_sync /*1500*/ + last_sync_length;




		if( is_new_pulse )
		{
			int is_master_sync_pulse = delta > so->pulse_in_clear_time /*40000*/; 
			int is_pulse_from_same_lh_as_last_sweep;
			int tp = delta % ( so->timecenter_ticks * 2);
			is_pulse_from_same_lh_as_last_sweep = tp < so->pulse_synctime_slack && tp > -so->pulse_synctime_slack;

			if( !so->did_handle_ootx )
			{
				HandleOOTX( ctx, so );
			}
			if( !is_master_sync_pulse )
			{
				so->did_handle_ootx = 0;
			}


			if( is_master_sync_pulse )  //Could also be called by slave if no master was seen.
			{
				ssn = so->sync_set_number = is_pulse_from_same_lh_as_last_sweep?(so->sync_set_number):0; //If repeated lighthouse, just back off one.
				if( ssn < 0 ) { SV_INFO( "SEVERE WARNING: Pulse codes for tracking not able to be backed out.\n" ); ssn = 0; }
				if( ssn != 0 )
				{
					//If it's the slave that is repeated, be sure to zero out its sync info.
					so->last_sync_length[0] = 0;
				}
				else
				{
					so->last_sync_length[1] = 0;
				}
				so->last_sync_time[ssn] = le->timestamp;
				so->last_sync_length[ssn] = le->length;
			}
			else if( so->sync_set_number == -1 )
			{
				//Do nothing.
			}
			else
			{
				ssn = ++so->sync_set_number;

				if( so->sync_set_number >= NUM_LIGHTHOUSES )
				{
					SV_INFO( "Warning.  Received an extra, unassociated sync pulse." );
					ssn = so->sync_set_number = -1;
				}
				else
				{
					so->last_sync_time[ssn] = le->timestamp;
					so->last_sync_length[ssn] = le->length;
				}
			}
		}
		else
		{
			//Find the longest pulse.
			if( le->length > last_sync_length )
			{
				if( so->last_sync_time[ssn] > le->timestamp )
				{
					so->last_sync_time[ssn] = le->timestamp;
					so->last_sync_length[ssn] = le->length;
				}
			}
		}

#if 0
		//Extra tidbit for storing length-of-sync-pulses, if you want to try to use this to determine AoI or distance to LH.
		//We don't actually use this anywhere, and I doubt we ever will?  Though, it could be useful at a later time to improve tracking.
		{
			int32_t main_divisor = so->timebase_hz / 384000; //125 @ 48 MHz.
			int base_station = is_new_pulse;
			printf( "%s %d %d %d\n", so->codename, le->sensor_id, so->sync_set_number, le->length ); //XXX sync_set_number is wrong here.
			ctx->lightproc( so, le->sensor_id, -3 - so->sync_set_number, 0, le->timestamp, le->length, base_station); //XXX sync_set_number is wrong here.
		}
#endif
	}

	//Any else- statements below here are 

	//See if this is a valid actual pulse.
	else if( le->length < so->pulse_max_for_sweep && delta > so->pulse_in_clear_time && ssn >= 0 )
	{
		int32_t dl = so->last_sync_time[0];
		int32_t tpco = so->last_sync_length[0];


#if NUM_LIGHTHOUSES != 2
		#error You are going to have to fix the code around here to allow for something other than two base stations.
#endif

		//Adding length 
		//Long pulse-code from IR flood.
		//Make sure it fits nicely into a divisible-by-500 time.

		int32_t main_divisor = so->timebase_hz / 384000; //125 @ 48 MHz.
		int acode = decode_acode(so->last_sync_length[0],main_divisor);

		if( !so->did_handle_ootx )
			HandleOOTX( ctx, so );

		int32_t offset_from = le->timestamp - dl + le->length/2;

		//Make sure pulse is in valid window
		if( offset_from < so->timecenter_ticks*2-so->pulse_in_clear_time && offset_from > so->pulse_in_clear_time )
		{
			int whichlh;
			if( acode < 0 ) whichlh = 1;
			else whichlh = !(acode>>2);
			ctx->lightproc( so, le->sensor_id, acode, offset_from, le->timestamp, le->length, whichlh );
		}
	}
	else
	{
		//printf( "FAIL %d   %d - %d = %d\n", le->length, so->last_photo_time, le->timestamp, so->last_photo_time - le->timestamp );
		//Runt pulse, or no sync pulses available.
	}
#endif
}



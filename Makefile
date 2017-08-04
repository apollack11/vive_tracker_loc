CC:=gcc
CFLAGS:=-Iinclude/libsurvive -I. -fPIC -g -O3 -Iredist -flto -DUSE_DOUBLE -std=gnu99 -rdynamic
LDFLAGS:=-L/usr/local/lib -lpthread -lz -lm -flto -g


LDFLAGS:=$(LDFLAGS) -lX11 -lusb-1.0
DRAWFUNCTIONS=redist/CNFGFunctions.c redist/CNFGXDriver.c
GRAPHICS_LOFI:=redist/CNFGFunctions.o redist/CNFGXDriver.o


POSERS:=src/poser_dummy.o src/poser_daveortho.o src/poser_charlesslow.o src/poser_octavioradii.o src/poser_turveytori.o src/poser_pollackradii.o src/poser_pollackpnp.o
REDISTS:=redist/json_helpers.o redist/linmath.o redist/jsmn.o redist/os_generic.o
LIBSURVIVE_CORE:=src/survive.o src/survive_usb.o src/survive_data.o src/survive_process.o src/ootx_decoder.o src/survive_driverman.o src/survive_vive.o src/survive_config.o src/survive_cal.o

LIBSURVIVE_O:=$(POSERS) $(REDISTS) $(LIBSURVIVE_CORE)
LIBSURVIVE_C:=$(LIBSURVIVE_O:.o=.c)

all : calibrate

calibrate :  lib/libsurvive.so calibrate.c redist/os_generic.c $(DRAWFUNCTIONS)
	$(CC) -o $@ calibrate.c redist/os_generic.c $(DRAWFUNCTIONS) $(LDFLAGS) -L./lib -lsurvive -lposecalc $(CFLAGS) -Wl,-rpath=./lib

lib :
	mkdir lib

lib/libposecalc.so : opencv_pose_calc.cpp
	g++ -o $@ $^ `pkg-config opencv --cflags --libs` -shared -fPIC

lib/libsurvive.so : lib/libposecalc.so $(LIBSURVIVE_O)
	$(CC) -o $@ $(LIBSURVIVE_O) $(LDFLAGS) -L./lib -lposecalc -shared 

clean :
	rm -rf *.o src/*.o *~ src/*~ calibrate lib/* redist/*.o redist/*~ 

.PHONY:
	all clean

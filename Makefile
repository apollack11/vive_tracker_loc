BASEFLAGS:=-Iinclude/libsurvive -Iinclude/libposecalc -I. -fPIC -g -O3 -Iredist -flto -DUSE_DOUBLE -rdynamic
CFLAGS:=$(BASEFLAGS) -std=gnu99
LDFLAGS:=-L/usr/local/lib -lpthread -lz -lm -flto -g -lX11 -lusb-1.0

POSERS:=src/poser_pollackpnp.o
REDISTS:=redist/json_helpers.o redist/linmath.o redist/jsmn.o redist/os_generic.o
LIBSURVIVE_CORE:=src/survive.o src/survive_data.o src/survive_process.o src/ootx_decoder.o src/survive_driverman.o src/survive_vive.o src/survive_config.o src/survive_cal.o

LIBSURVIVE_O:=$(POSERS) $(REDISTS) $(LIBSURVIVE_CORE)
LIBSURVIVE_C:=$(LIBSURVIVE_O:.o=.c)

all : runposer

runposer : lib/libsurvive.so src/runposer.c redist/os_generic.c
	g++ -o $@ src/runposer.c redist/os_generic.c $(LDFLAGS) -L./lib -lsurvive -lposecalc $(BASEFLAGS) -Wl,-rpath=./lib

lib :
	mkdir $@

lib/libposecalc.so : lib src/opencv_pose_calc.cpp
	g++ -o $@ src/opencv_pose_calc.cpp -Iinclude/libsurvive -Iinclude/libposecalc `pkg-config opencv --cflags --libs` -shared -fPIC

lib/libsurvive.so : lib/libposecalc.so $(LIBSURVIVE_O)
	g++ -o $@ $(LIBSURVIVE_O) $(LDFLAGS) -L./lib -lposecalc -shared 

clean :
	rm -rf *.o src/*.o *~ src/*~ runposer lib/* redist/*.o redist/*~ 

.PHONY:
	all clean

cmake_minimum_required(VERSION 2.8.3)
project(vive_tracker_loc)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

find_package(OpenCV REQUIRED)
# find_package(libusb-1.0 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)

include_directories(include
  include/survive
  include/posecalc
  ${OpenCV_INCLUDE_DIRS}
  ${LIBUSB1_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
catkin_package()

set(CMAKE_C_FLAGS "-std=gnu99")

add_library(survive SHARED
  src/ootx_decoder.c
  src/poser_pollackpnp.c
  src/runposer.c
  src/survive.c
  src/survive_cal.c
  src/survive_config.c
  src/survive_data.c
  src/survive_driverman.c
  src/survive_process.c
  src/survive_vive.c
  src/redist/jsmn.c  
  src/redist/json_helpers.c  
  src/redist/linmath.c  
  src/redist/os_generic.c  
  )

add_library(posecalc SHARED
  src/opencv_pose_calc.cpp
  )
target_link_libraries(posecalc ${OpenCV_LIBRARIES})

add_executable(runposer src/runposer.c)
target_link_libraries(runposer survive posecalc ${OpenCV_LIBRARIES} ${LIBUSB1_LIBRARIES})

add_executable(vive_pose_node src/vive_pose_node.cpp)
target_link_libraries(vive_pose_node survive posecalc ${OpenCV_LIBRARIES} ${LIBUSB1_LIBRARIES} ${catkin_LIBRARIES})

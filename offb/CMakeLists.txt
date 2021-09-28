cmake_minimum_required(VERSION 3.0.2)
project(offb)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
 FILES
 obj.msg
 objs.msg
 frame.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs message_generation
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(camera
    src/camera.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(camera
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(avoid
    src/rca.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(avoid
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(avoid1
    src/rca1.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(avoid1
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(avoid2
    src/rca2.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(avoid2
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(cood
    src/cood.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(cood
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(gps
    src/gps.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(gps
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(yolo
    src/yolo.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(yolo
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(yl0
    src/yl0.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(yl0
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(yl1
    src/yl1.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(yl1
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(yl2
    src/yl2.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(yl2
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(fj005
    src/fj005.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(fj005
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(tracking
    src/tracking.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(tracking
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(trial
    src/trial.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(trial
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(vicon0
    src/vicon0.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(vicon0
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
add_executable(vicon1
    src/vicon1.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(vicon1
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
add_executable(coodg
    src/coodg.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(coodg
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
add_executable(coodv
    src/coodv.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(coodv
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
add_executable(vm0
    src/vm0.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(vm0
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
add_executable(vm1
    src/vm1.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(vm1
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

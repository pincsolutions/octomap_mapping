cmake_minimum_required(VERSION 2.8)
project(octomap_server)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  nodelet  
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

find_package(octomap REQUIRED)

add_definitions(-DOCTOMAP_NODEBUGOUT -Wall -O3)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 11)

#add_definitions (-Wall -O3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -funsafe-loop-optimizations -fsee -funroll-loops -fno-math-errno -funsafe-math-optimizations -ffinite-math-only -fno-signed-zeros ")

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
  message("enabling msse2 for x86_64 or x86_32 architecture")
  #  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse2 ")
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm)
  message("enabling -mfpu=neon -mfloat-abi=softfp for ARM architecture")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mcpu=cortex-a9 -mfpu=neon -mtune=cortex-a9 -mvectorize-with-neon-quad -ffast-math ")
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)


generate_dynamic_reconfigure_options(cfg/OctomapServer.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS octomap PCL
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME} src/OctomapServer.cpp src/OctomapServerMultilayer.cpp src/TrackingOctomapServer.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(octomap_server_node src/octomap_server_node.cpp)
target_link_libraries(octomap_server_node ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(octomap_color_server_node src/octomap_color_server_node.cpp)
#target_link_libraries(octomap_color_server_node ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_server_static src/octomap_server_static.cpp)
target_link_libraries(octomap_server_static ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_server_multilayer src/octomap_server_multilayer.cpp)
target_link_libraries(octomap_server_multilayer ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_saver src/octomap_saver.cpp)
target_link_libraries(octomap_saver ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_tracking_server_node src/octomap_tracking_server_node.cpp)
target_link_libraries(octomap_tracking_server_node ${PROJECT_NAME} ${LINK_LIBS})

# Nodelet
add_library(octomap_server_nodelet src/octomap_server_nodelet.cpp)
target_link_libraries(octomap_server_nodelet ${PROJECT_NAME} ${LINK_LIBS})

# install targets:
install(TARGETS ${PROJECT_NAME}
  octomap_server_node
  octomap_server_static
  octomap_server_multilayer
  octomap_saver
  octomap_tracking_server_node
  octomap_server_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
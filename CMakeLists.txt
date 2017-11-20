cmake_minimum_required(VERSION 2.4.6)
project(dso_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  cv_bridge
)

# TODO: add cmake config files to dso such that it is picked up automatically, like pangolin!

##########################################################################
# You can set DSO_PATH to your source folder by passing
# `-DDSO_PATH=...`, setting environment variables, or by uncommenting
# the following lines. DSO_LIBRARY_PATH defaults to "${DSO_PATH}/build/lib.
#
# If DSO_PATH is not set, we'll try to find a DSO install with cmake
# (which also works in works with DSO installed in the devel folder).
##########################################################################

#set(DSO_PATH         /home/user/dso)
#set(DSO_LIBRARY_PATH /home/user/build-dso)

# set values from environment variables
if (NOT DSO_PATH)
    set(DSO_PATH $ENV{DSO_PATH})
endif()
if (NOT DSO_LIBRARY_PATH)
    set(DSO_LIBRARY_PATH $ENV{DSO_LIBRARY_PATH})
    if (NOT DSO_LIBRARY_PATH AND DSO_PATH)
        set(DSO_LIBRARY_PATH "${DSO_PATH}/build/lib")
    endif()
endif()

if (DSO_PATH)
    # Manually set path to DSO source folder
    message("DSO_PATH set, trying to find library.")
    message("---- EXPECTING DSO sources at\n\"${DSO_PATH}\"")
    set(DSO_INCLUDE_DIRS "${DSO_PATH}/src" "${DSO_PATH}/thirdparty/Sophus")
    message("---- LOOKING FOR DSO library at\n\"${DSO_LIBRARY_PATH}\"")
    find_library(DSO_LIBRARY dso ${DSO_LIBRARY_PATH})
else()
    # Detect DSO install (also catkin devel folder)
    message("DSO_PATH not set yet, trying to find installed dso headers and library.")
    find_path(DSO_INCLUDE_DIRS dso)
    if (DSO_INCLUDE_DIRS)
        set(DSO_INCLUDE_DIRS "${DSO_INCLUDE_DIRS}/dso")
        message("---- FOUND DSO headers at \"${DSO_INCLUDE_DIRS}\"")
    endif()
    find_library(DSO_LIBRARY dso)
endif()


if (NOT DSO_INCLUDE_DIRS)
    message(FATAL_ERROR "DSO headers not found and/or DSO_PATH not set.")
endif()
if (NOT DSO_LIBRARY)
    message(FATAL_ERROR "DSO library not found and/or DSO_LIBRARY_PATH not set.")
endif()

message("---- Found DSO library at \"${DSO_LIBRARY}\"")



find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    cv_bridge
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(dso_live src/main.cpp)

target_link_libraries(dso_live
    ${DSO_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})

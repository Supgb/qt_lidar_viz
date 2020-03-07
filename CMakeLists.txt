## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the
## TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(qt_lidar_viz)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
find_package(catkin REQUIRED COMPONENTS
    rviz roscpp rospy
    laser_geometry
    sensor_msgs
    tf
    pcl_ros
    pcl_conversions
    pluginlib
    std_msgs
    visualization_msgs
    message_generation
    )
find_package(Boost REQUIRED)
find_package(PCL 1.2 REQUIRED)

add_message_files(
    FILES
    LslidarN301Packet.msg
    LslidarN301Point.msg
    LslidarN301Scan.msg
    LslidarN301Sweep.msg
        )

generate_messages(
    DEPENDENCIES
    std_msgs
    )

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        std_msgs roscpp sensor_msgs visualization_msgs
        pluginlib tf
        pcl_ros pcl_conversions
    DEPENDS
        Boost
        )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    )
link_directories(
    ${rviz_DEFAULT_PLUGIN_LIBRARIES}
    ${catkin_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
    )
add_definitions(
    ${PCL_DEFINITIONS}
    )

## Lslidar driver.
add_library(lslidar_n301_driver
    src/lslidar_n301_driver.cc)
target_link_libraries(lslidar_n301_driver
    ${catkin_LIBRARIES}
    )
add_dependencies(lslidar_n301_driver
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

## Lslidar decode driver.
add_library(lslidar_n301_decoder
    src/lslidar_n301_decoder.cpp
    )
target_link_libraries(lslidar_n301_decoder
    ${catkin_LIBRARIES}
    )
add_dependencies(lslidar_n301_decoder
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )


## This setting causes Qt's "MOC" generation to happen automatically.
## set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

QT5_WRAP_CPP(MOC_FILES
    include/myviz.h
    include/lidar_assembler.h
    include/lidarctrlpanel.h
    include/lidarinfo.h
    #include/lidartune.h
    include/lidarmdl.h
    include/logsys.h
    include/lidardriver.h
    include/lidardecoder.h
    include/lidardebug.h
    )

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/myviz.cpp
  src/main.cpp
  src/lidar_assembler.cpp
  src/lidarctrlpanel.cpp
  src/lidarinfo.cpp
  src/lidartune.cpp
  src/lidarmdl.cpp
  src/logsys.cpp
  src/lidardriver.cpp
  src/lidardecoder.cpp
  src/lidardebug.cpp
)

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SRC_FILES}``.
add_executable(myviz ${SRC_FILES} ${MOC_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
target_link_libraries(myviz
    lslidar_n301_driver
    lslidar_n301_decoder
    ${QT_LIBRARIES}
    ${rviz_DEFAULT_PLUGIN_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})
add_dependencies(myviz
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
## END_TUTORIAL

## Install
install(TARGETS myviz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

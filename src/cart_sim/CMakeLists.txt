cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_AUTOMOC ON)
project(cart_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  joy
  message_generation
  #pcl için :
  pcl_conversions
	pcl_ros
  dynamic_reconfigure
  roslint
)

find_package(gazebo 7 QUIET)
if (gazebo_FOUND)
  message(STATUS "gazebo7 found!")
else()
  message(STATUS "gazebo7 not found, searching for gazebo8")
  find_package(gazebo 8 QUIET)
  if (gazebo_FOUND)
    message(STATUS "gazebo8 found!")
  else()
    message(STATUS "gazebo8 not found, searching for gazebo9")
    find_package(gazebo 9 QUIET)
    if (gazebo_FOUND)
      message(STATUS "gazebo9 found!")
    else()
      message(STATUS "gazebo9 not found, searching for gazebo10")
      find_package(gazebo 11 QUIET)
    endif()
  endif()
endif()
if (NOT gazebo_FOUND)
  message(FATAL_ERROR "Could not find a version of gazebo >= 7")
endif()


find_package(Qt5Widgets REQUIRED)
find_package(Qt5X11Extras REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost  REQUIRED COMPONENTS thread )

add_message_files(
  FILES
  cart_control.msg
)



generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs gazebo_ros message_runtime 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)



include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${Qt5Core_INCLUDE_DIRS} ${Qt5X11Extras_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_definitions(${PCL_DEFINITIONS})

#cloud_transformer çalıştırılabilir dosyasını ekle
add_executable(cloud_transformer src/cloud_transformer.cpp)
target_link_libraries(cloud_transformer ${catkin_LIBRARIES})

add_library(cartplugin SHARED plugins/CartPlugin.cc)
target_link_libraries(cartplugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(cartplugin ${catkin_EXPORTED_TARGETS})
add_library(kbplug SHARED plugins/KeyboardGUIPlugin.cc)
target_link_libraries(kbplug ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(kbplug ${catkin_EXPORTED_TARGETS})
catkin_install_python(PROGRAMS nodes/controller
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




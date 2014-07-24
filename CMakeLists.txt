cmake_minimum_required(VERSION 2.8.3)
project(srl_smp)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  daryl_msgs
)



# add qt4 to the project
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
include_directories(${Eigen_INCLUDE_DIRS})



catkin_package(

  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf visualization_msgs 
)


###########
## Build ##
###########



INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/

)

## Declare a cpp library
add_library(KDTREE_LIB ${CMAKE_CURRENT_SOURCE_DIR}/src/smp/external_libraries/kdtree/kdtree.c)


set(SOURCES 
    src/srl_smp.cpp src/trajectory.cpp src/orientationhandler.cpp src/orientationhandler.h 

) 



add_executable(srl_smp ${SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(srl_smp ${catkin_EXPORTED_TARGETS})

add_executable(steer_robot src/steer_robot.cpp src/steer_robot.h   src/orientationhandler.cpp src/orientationhandler.h )
add_dependencies(steer_robot ${catkin_EXPORTED_TARGETS})

TARGET_LINK_LIBRARIES(srl_smp 
    ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QWT_LIBRARIES}  
    ${BOOST_LIBRARIES}  KDTREE_LIB  ${catkin_LIBRARIES}
)

TARGET_LINK_LIBRARIES(steer_robot ${BOOST_LIBRARIES} ${catkin_LIBRARIES})



INSTALL(TARGETS srl_smp  steer_robot
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

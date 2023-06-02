cmake_minimum_required(VERSION 3.0.2)
project(ompl_near_projection)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  )
find_package(ompl REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3>=3.2.7)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_near_projection
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS OMPL EIGEN
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
 ${EIGEN_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/NearProjectedStateSpace.cpp
  src/NearConstrainedSpaceInformation.cpp
  src/NearProblemDefinition.cpp
  src/NearGoalSpace.cpp
  src/NearKPIECE1.cpp
  src/NearEST.cpp
  src/pNearKPIECE1.cpp
  src/NearRRTConnect.cpp
  src/pNearRRTConnect.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

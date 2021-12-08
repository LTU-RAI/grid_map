cmake_minimum_required(VERSION 3.0.2)
project(occupancy_invert)

  # Get the information about this package's buildtime dependencies
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
  )

  # Declare that this catkin package's runtime dependencies
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp nav_msgs
  )

  include_directories(
      #include/occupancy_grid_invers.hpp
      include
      ${catkin_INCLUDE_DIRS}
  )

  add_library(${PROJECT_NAME}
      src/occupancy_grid_invers.cpp
  )

  # define executable not using any messages/services provided by this package
  add_executable(${PROJECT_NAME}_node src/occupancy_grid_invers.cpp)

  target_link_libraries(${PROJECT_NAME}_node
      ${PROJECT_NAME}
     ${catkin_LIBRARIES}
  )

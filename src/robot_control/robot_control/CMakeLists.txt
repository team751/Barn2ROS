include_directories(
  include/
)

# Robot Hardware Interface
add_library(robot_hw_interface
  src/robot_hw_interface.cpp
)
target_link_libraries(robot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(robot_hw_main src/robot_hw_main.cpp)
target_link_libraries(robot_hw_main
  robot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    robot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    robot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/robot_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

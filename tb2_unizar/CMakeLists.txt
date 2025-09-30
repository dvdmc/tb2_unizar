cmake_minimum_required(VERSION 3.5)
project(tb2_unizar)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies NO NODE FOR NOW
# set(PROJECT_DEPENDENCIES
#   ament_cmake
#   rclcpp
#   rclcpp_components
# )
# foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
#   find_package(${DEPENDENCY} REQUIRED)
# endforeach()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

# INCLUDE IF USE NODE
# ament_package()
cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME latlon2utm)
project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies

set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  sensor_msgs
  geometry_msgs
  GeographicLib
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(SOURCE_CPP_FILES 
  src/GpsUtils.cpp
)

include_directories(
  include
)

add_executable(${PROJECT_NAME} src/main.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})


if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()


# add_executable(latlon2utm ${SOURCE_FILES})
# target_link_libraries (latlon2utm ${GeographicLib_LIBRARIES})
# target_link_libraries (latlon2utm ${geometry_msgs_LIBRARIES})
# target_link_libraries (latlon2utm ${sensor_msgs_LIBRARIES})

add_executable(other src/other.cpp)

add_executable(tests src/tests.cpp)
target_link_libraries (tests ${GeographicLib_LIBRARIES})

# https://stackoverflow.com/questions/48169653/finding-geographiclib-in-cmake-on-debian
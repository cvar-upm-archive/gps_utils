cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME gps_utils)
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
  as2_core
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
  include/${PROJECT_NAME}
)

add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# TEMPORAL
add_executable(gps_translator src/GpsTranslator.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(gps_translator ${PROJECT_DEPENDENCIES})

add_executable(${PROJECT_NAME}_main src/main.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_main ${PROJECT_DEPENDENCIES})


if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)
endif()

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
cmake_minimum_required(VERSION 3.8)
project(libmpc_plugin)

# Enable C++20
set(CMAKE_CXX_STANDARD 20)
# Enable optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
# Disable eigen stack allocation warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=0")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# Find the mpc++ package
find_package(mpc++ CONFIG REQUIRED)

include_directories(
  include
)
# Include the mpc++ headers
include_directories(${mpc++_INCLUDE_DIRS})

install(
  DIRECTORY include/
  DESTINATION include
)

add_library(libmpc_plugin SHARED src/nmpc_planner.cpp)
target_include_directories(libmpc_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
    libmpc_plugin
    pluginlib
    mpc++
  )  

pluginlib_export_plugin_description_file(libmpc_plugin plugins.xml)  

install(
  TARGETS libmpc_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

target_link_libraries(${PROJECT_NAME} mpc++ Eigen3::Eigen)

ament_export_libraries(
  libmpc_plugin
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

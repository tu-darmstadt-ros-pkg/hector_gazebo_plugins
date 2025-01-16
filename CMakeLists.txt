cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(hector_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake4 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin3 REQUIRED COMPONENTS register)

set(GZ_PLUGIN_VER ${gz-plugin3_VERSION_MAJOR})
include_directories(include include)


# Add new plugins here
add_library(MimicJoints SHARED src/MimicJoints.cc)
set_property(TARGET MimicJoints PROPERTY CXX_STANDARD 17)
target_link_libraries(MimicJoints
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim8::gz-sim8)

install(TARGETS MimicJoints
  LIBRARY DESTINATION plugins
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks(hooks/${PROJECT_NAME}.dsv.in)
ament_environment_hooks(hooks/${PROJECT_NAME}.sh.in)

ament_package()

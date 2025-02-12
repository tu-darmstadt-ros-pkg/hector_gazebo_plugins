cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(hector_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(xacro REQUIRED)

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
include_directories(include include)

set(PLACEHOLDER_XACRO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/xacro/plugin_placeholder.xacro)
set(GLOBAL_PLUGIN_XACRO ${CMAKE_CURRENT_SOURCE_DIR}/xacro/global_plugins.xacro)
set(OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/plugin_placeholder.sdf)

# Command to generate SDF from XACRO
add_custom_command(
    OUTPUT ${OUTPUT_FILE}
    COMMAND xacro ${PLACEHOLDER_XACRO_FILE} -o ${OUTPUT_FILE}
    DEPENDS ${PLACEHOLDER_XACRO_FILE} ${GLOBAL_PLUGIN_XACRO}
    COMMENT "Generating placeholder sdf from Xacro"
    VERBATIM
)

# Create a target that depends on the generated file
add_custom_target(generate_sdf ALL DEPENDS ${OUTPUT_FILE})

# Install the generated SDF
install(FILES ${OUTPUT_FILE} DESTINATION share/${PROJECT_NAME}/plugin_placeholder_sdf)


# Add new plugins here which need to be built
add_library(MimicJoints SHARED src/MimicJoints.cc)
set_property(TARGET MimicJoints PROPERTY CXX_STANDARD 17)
target_link_libraries(MimicJoints
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim8::gz-sim8)

install(TARGETS MimicJoints
  LIBRARY DESTINATION plugins
)

install(DIRECTORY
  launch
  xacro
  DESTINATION share/${PROJECT_NAME})

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

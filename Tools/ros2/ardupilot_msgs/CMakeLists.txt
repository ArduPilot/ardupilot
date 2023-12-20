cmake_minimum_required(VERSION 3.8)
project(ardupilot_msgs)

# --------------------------------------------------------------------------- #
# Find dependencies.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------------------------------- #
# Generate and export message interfaces.

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmMotors.srv"
  "srv/ModeSwitch.srv"
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------------------- #
# Call last.

ament_package()

cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuturtle_control REQUIRED)
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

if(NOT CMAKE_CROSSCOMPILING)
    # CMake also has the ability to generate doxygen documentation
    find_package(Doxygen)
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
    doxygen_add_docs(doxygen src/ README.md ALL)
endif()

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

add_executable(slam src/slam.cpp)
ament_target_dependencies(slam rclcpp sensor_msgs nav_msgs tf2_ros tf2 geometry_msgs nuturtle_control visualization_msgs)
target_link_libraries(slam turtlelib::turtlelib "${cpp_typesupport_target}" ${ARMADILLO_LIBRARIES})

add_executable(landmarks src/landmarks.cpp)
ament_target_dependencies(landmarks rclcpp sensor_msgs nav_msgs tf2_ros tf2 geometry_msgs nuturtle_control visualization_msgs)
target_link_libraries(landmarks turtlelib::turtlelib "${cpp_typesupport_target}" ${ARMADILLO_LIBRARIES})

install(TARGETS
  landmarks
  slam
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

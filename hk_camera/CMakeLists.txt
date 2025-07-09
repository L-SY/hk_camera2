cmake_minimum_required(VERSION 3.8)
project(hk_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 添加海康SDK头文件路径
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include
)

# 添加海康SDK库文件路径
link_directories(/opt/MVS/lib/64)

# Source files (排除main文件，避免重复定义)
file(GLOB SOURCES "src/camera_manager.cpp" "src/hk_camera_node.cpp" "src/hk_camera_stitching_node.cpp")

# Create library
add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} MvCameraControl yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs cv_bridge yaml-cpp ament_index_cpp)

# 添加test_single_camera可执行文件 (commented out due to missing file)
# add_executable(test_single_camera test/test_single_camera.cpp)
# target_link_libraries(test_single_camera ${PROJECT_NAME} ${OpenCV_LIBS} MvCameraControl yaml-cpp)
# install(TARGETS test_single_camera DESTINATION lib/${PROJECT_NAME})

# 新增主节点可执行文件
add_executable(hk_camera_node_main src/hk_camera_node_main.cpp)
target_link_libraries(hk_camera_node_main ${PROJECT_NAME} ${OpenCV_LIBS} MvCameraControl yaml-cpp)
ament_target_dependencies(hk_camera_node_main rclcpp sensor_msgs cv_bridge yaml-cpp)
install(TARGETS hk_camera_node_main DESTINATION lib/${PROJECT_NAME})

# 新增拼接节点可执行文件
add_executable(hk_camera_stitching_node_main src/hk_camera_stitching_node_main.cpp)
target_link_libraries(hk_camera_stitching_node_main ${PROJECT_NAME} ${OpenCV_LIBS} MvCameraControl yaml-cpp)
ament_target_dependencies(hk_camera_stitching_node_main rclcpp sensor_msgs cv_bridge yaml-cpp ament_index_cpp)
install(TARGETS hk_camera_stitching_node_main DESTINATION lib/${PROJECT_NAME})

# Test executables commented out due to missing files
# add_executable(fps_analysis_test test/fps_analysis_test.cpp src/camera_manager.cpp)
# target_include_directories(fps_analysis_test PRIVATE include)
# target_link_libraries(fps_analysis_test ${OpenCV_LIBS} MvCameraControl yaml-cpp)
# ament_target_dependencies(fps_analysis_test rclcpp sensor_msgs cv_bridge)

# add_executable(dual_camera_fps_test test/dual_camera_fps_test.cpp src/camera_manager.cpp)
# target_include_directories(dual_camera_fps_test PRIVATE include)
# target_link_libraries(dual_camera_fps_test ${OpenCV_LIBS} MvCameraControl yaml-cpp)
# ament_target_dependencies(dual_camera_fps_test rclcpp sensor_msgs cv_bridge)

# Install test executables
# install(TARGETS fps_analysis_test dual_camera_fps_test
#   DESTINATION lib/${PROJECT_NAME})

# 安装头文件和库
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY files
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Testing related
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

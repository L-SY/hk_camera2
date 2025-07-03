cmake_minimum_required(VERSION 3.8)
project(hk_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# 添加海康SDK头文件路径
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include
)

# 添加海康SDK库文件路径
link_directories(/opt/MVS/lib/64)

# Source files
file(GLOB SOURCES "src/*.cpp")

# Create library
add_library(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} MvCameraControl)

# 添加test_single_camera可执行文件
add_executable(test_single_camera test/test_single_camera.cpp)
target_link_libraries(test_single_camera ${PROJECT_NAME} ${OpenCV_LIBS} MvCameraControl)
install(TARGETS test_single_camera DESTINATION lib/${PROJECT_NAME})

# 安装头文件和库
install(
  DIRECTORY include/
  DESTINATION include/
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

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(hk_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# Add include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Source files
file(GLOB SOURCES "src/*.cpp")

# Create library
add_library(${PROJECT_NAME} ${SOURCES})

# Link dependencies (add other dependencies here if needed)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# Install header files and library
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

ament_package()

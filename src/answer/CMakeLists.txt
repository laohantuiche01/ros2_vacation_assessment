cmake_minimum_required(VERSION 3.8)
project(answer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(answer_infos REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include)
include_directories(include msg_serializer thirdparty/serialib)

add_subdirectory(thirdparty/serialib)

ament_auto_add_executable(example_test src/test.cpp)
target_link_libraries(example_test serialib ${OpenCV_LIBS})

ament_auto_add_executable(${PROJECT_NAME}_img src/image_handle/img_handle.cpp)
target_link_libraries(${PROJECT_NAME}_img serialib ${OpenCV_LIBS})

ament_auto_add_executable(${PROJECT_NAME}_algo src/Algorithm.cpp)
target_link_libraries(${PROJECT_NAME}_algo serialib ${OpenCV_LIBS})

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

ament_auto_package()


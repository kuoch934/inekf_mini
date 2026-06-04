cmake_minimum_required(VERSION 3.8)
project(inEKF_mini)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)



add_library(inEKF_mini_interface INTERFACE)
target_include_directories(inEKF_mini_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_subdirectory(inekf_mini)

add_executable(cassie_inekf_app src/cassie_inekf_main.cpp)
target_link_libraries(cassie_inekf_app
    PRIVATE
    inEKF_mini_interface
    inekf_mini            # 鏈接你寫的 InEKF 數學核心
    pinocchio::pinocchio  # 鏈接運動學庫
    Eigen3::Eigen         # 鏈接矩陣運算庫
    rclcpp::rclcpp        # 鏈接 ROS 2 C++ 客戶端庫
)

# 安裝執行檔到 lib/${PROJECT_NAME} 目錄
install(TARGETS 
  cassie_inekf_app 
  DESTINATION lib/${PROJECT_NAME}
)

# 如果你希望執行時能讀到 URDF 和數據，也可以順便安裝資源 (選配)
install(DIRECTORY cassie_description data
  DESTINATION share/${PROJECT_NAME}
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

ament_package()

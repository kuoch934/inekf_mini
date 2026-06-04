set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(inekf_mini LANGUAGES CXX)

find_package(Eigen3 REQUIRED)

add_library(inekf_mini STATIC
    src/robot_state.cpp
    src/propagation.cpp
    src/correction.cpp
)
target_link_libraries(inekf_mini PUBLIC Eigen3::Eigen)
target_include_directories(inekf_mini PUBLIC include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
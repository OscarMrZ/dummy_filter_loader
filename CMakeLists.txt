cmake_minimum_required(VERSION 3.8)
project(dummy_filter_loader)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Create executable
ament_auto_add_executable(dummy_filter_loader src/filter_loader.cpp)

if(BUILD_TESTING)
endif()

ament_auto_package()

cmake_minimum_required(VERSION 3.8)
project(muscle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
# library target: lib_signal
# library to generate standard signals

# create target
add_library(lib_signal SHARED
  src/lib_signal.cpp
) # source file of the target
# add header directories of target
target_include_directories( lib_signal
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# add dependencies of the target
# e.g. target_link_libraries(my_library PUBLIC Eigen3::Eigen)
target_link_libraries(lib_signal PUBLIC Eigen3::Eigen)
# export the header of the library
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
# compile the library
install(
        TARGETS lib_signal
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# create target

add_executable(test_lib_signal
  src/test_lib_signal.cpp
)

# add dependencies inside package
target_include_directories(test_lib_signal
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(test_lib_signal lib_signal)

# install exetable
install(TARGETS test_lib_signal
    DESTINATION lib/${PROJECT_NAME})

ament_package()

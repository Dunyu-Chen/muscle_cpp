cmake_minimum_required(VERSION 3.8)
project(muscle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set build type to debug mode
SET(CMAKE_BUILD_TYPE "Debug")

# setup libtorch
set(CMAKE_PREFIX_PATH "/opt/libtorch/libtorch-cxx11-abi-shared-with-deps-2.3.1+cpu/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# basic dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# library target: matplotlib
# plot library using matplotlib from python
add_library(matplotlib_cpp INTERFACE)
target_include_directories( matplotlib_cpp
        INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(matplotlib_cpp INTERFACE
        cxx_std_11
)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
        Python3::Python
        Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
          Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
        TARGETS matplotlib_cpp
        EXPORT install_targets
)

# library target: lib_signal
# library to generate standard signals
# 1. create target
add_library(lib_signal SHARED
  src/lib_signal.cpp
)
# 2. add header directories of target
target_include_directories( lib_signal
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# 3. add dependencies of the target
target_link_libraries(lib_signal PUBLIC Eigen3::Eigen)
# 4. export the header of the library
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
# 5. compile the library
install(
        TARGETS lib_signal
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)
# 6. export the library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


# executable target: test_lib_signal
# test the lib_signal

# 1. create target
add_executable(test_lib_signal
  src/test_lib_signal.cpp
)

# 2. add dependencies inside package by header
target_include_directories(test_lib_signal
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# 3. add dependencies
target_link_libraries(test_lib_signal lib_signal)
target_link_libraries(test_lib_signal matplotlib_cpp)

target_link_libraries(test_lib_signal "${TORCH_LIBRARIES}")
set_property(TARGET test_lib_signal PROPERTY CXX_STANDARD 17)

# 4. install executable
install(TARGETS test_lib_signal
    DESTINATION lib/${PROJECT_NAME})


# executable target: linear_simulator
# ros node for linear system simulator

# 1. create target
add_executable(linear_simulator
        src/linear_simulator.cpp)
# 2. add dependencies inside package by header
target_include_directories(linear_simulator
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# 3. add dependencies
target_link_libraries(linear_simulator lib_signal)

find_package(std_msgs REQUIRED)

ament_target_dependencies(linear_simulator rclcpp std_msgs)
# 4. install executable
install(TARGETS linear_simulator
  DESTINATION lib/${PROJECT_NAME})

# executable target: signal_sin_gen
# ros node to generate sin signal

# 1. create target
add_executable(signal_sin_gen
        src/signal_sin_generator.cpp)
# 2. add dependencies inside package by header
target_include_directories(signal_sin_gen
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# 3. add dependencies
target_link_libraries(signal_sin_gen lib_signal)
find_package(std_msgs REQUIRED)
ament_target_dependencies(signal_sin_gen rclcpp std_msgs)
# 4. install executable
install(TARGETS signal_sin_gen
        DESTINATION lib/${PROJECT_NAME})

# executable target: pid_controller
# ros node for PID controller

# 1. create target
add_executable(pid_controller
        src/pid_controller.cpp)
# 2. add dependencies inside package by header
target_include_directories(pid_controller
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# 3. add dependencies
target_link_libraries(pid_controller lib_signal)
find_package(std_msgs REQUIRED)
ament_target_dependencies(pid_controller rclcpp std_msgs)
# 4. install executable
install(TARGETS pid_controller
        DESTINATION lib/${PROJECT_NAME})

# library target: lib_muscle
# library for pneumatic muscle
# 1. create target
add_library(lib_muscle SHARED
        src/lib_muscle.cpp
)
# 2. add header directories of target
target_include_directories( lib_muscle
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# 3. add dependencies of the target
target_link_libraries(lib_muscle PUBLIC Eigen3::Eigen)
# 4. export the header of the library
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
# 5. compile the library
install(
        TARGETS lib_muscle
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)
# 6. export the library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


ament_package()

cmake_minimum_required(VERSION 3.16)
project(whistle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_path(ALSA_INCLUDE_DIR alsa/asoundlib.h REQUIRED)
find_library(ALSA_LIBRARY NAMES asound REQUIRED)

set(TFLITE_THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/tflite)

if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  message(FATAL_ERROR "whistle_detector currently bundles only x86_64 TensorFlow Lite static libraries.")
endif()

add_library(tensorflow-lite STATIC IMPORTED)
add_library(ruy STATIC IMPORTED)
add_library(farmhash STATIC IMPORTED)
add_library(flatbuffers STATIC IMPORTED)
add_library(fft2d_fftsg STATIC IMPORTED)
add_library(fft2d_fftsg2d STATIC IMPORTED)
add_library(xnnpack STATIC IMPORTED)
add_library(cpuinfo STATIC IMPORTED)
add_library(clog STATIC IMPORTED)
add_library(pthreadpool STATIC IMPORTED)

set_target_properties(tensorflow-lite PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libtensorflow-lite.a)
set_target_properties(ruy PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libruy.a)
set_target_properties(farmhash PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libfarmhash.a)
set_target_properties(flatbuffers PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libflatbuffers.a)
set_target_properties(fft2d_fftsg PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libfft2d_fftsg.a)
set_target_properties(fft2d_fftsg2d PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libfft2d_fftsg2d.a)
set_target_properties(xnnpack PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libXNNPACK.a)
set_target_properties(cpuinfo PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libcpuinfo.a)
set_target_properties(clog PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libclog.a)
set_target_properties(pthreadpool PROPERTIES IMPORTED_LOCATION ${TFLITE_THIRDPARTY_DIR}/x64/libpthreadpool.a)
set_target_properties(tensorflow-lite PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${TFLITE_THIRDPARTY_DIR})

add_library(${PROJECT_NAME}_core
  src/tflite_executer.cpp
  src/tflite_whistle_detection.cpp
  src/alsa_whistle_detector.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TFLITE_THIRDPARTY_DIR}
  ${ALSA_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}_core
  tensorflow-lite
  ruy
  farmhash
  flatbuffers
  fft2d_fftsg
  fft2d_fftsg2d
  xnnpack
  cpuinfo
  clog
  pthreadpool
  Boost::filesystem
  ${ALSA_LIBRARY}
  ${CMAKE_DL_LIBS}
  m
  pthread
  rt
)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

add_executable(whistle_detector_node src/whistle_detector_node.cpp)
target_link_libraries(whistle_detector_node ${PROJECT_NAME}_core)
ament_target_dependencies(whistle_detector_node rclcpp std_msgs ament_index_cpp)
target_compile_features(whistle_detector_node PUBLIC cxx_std_17)

install(TARGETS
  ${PROJECT_NAME}_core
  whistle_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY data launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

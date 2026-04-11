cmake_minimum_required(VERSION 3.16)
project(whistle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_path(ALSA_INCLUDE_DIR alsa/asoundlib.h REQUIRED)
find_library(ALSA_LIBRARY NAMES asound REQUIRED)
find_path(FFTW3_INCLUDE_DIR fftw3.h)
find_library(FFTW3F_LIBRARY NAMES fftw3f)

set(TFLITE_THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/tflite)
set(TFLITE_X64_DIR ${TFLITE_THIRDPARTY_DIR}/x64)
set(TFLITE_LIBRARY "" CACHE FILEPATH "Path to a TensorFlow Lite library for non-bundled builds")
set(TFLITE_INCLUDE_DIR "" CACHE PATH "Additional TensorFlow Lite include path for non-bundled builds")
option(WHISTLE_DETECTOR_ALLOW_STUB_FALLBACK "Build a disabled stub if no TensorFlow Lite library is available" ON)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  option(WHISTLE_DETECTOR_ENABLE_XNNPACK "Enable the TensorFlow Lite XNNPACK delegate" ON)
else()
  option(WHISTLE_DETECTOR_ENABLE_XNNPACK "Enable the TensorFlow Lite XNNPACK delegate" OFF)
endif()

if(NOT TFLITE_INCLUDE_DIR)
  find_path(TFLITE_INCLUDE_DIR_AUTO
    NAMES tflite_c_api.h tensorflow/lite/c/c_api.h
  )
  if(TFLITE_INCLUDE_DIR_AUTO)
    set(TFLITE_INCLUDE_DIR ${TFLITE_INCLUDE_DIR_AUTO})
  endif()
endif()

set(WHISTLE_DETECTOR_TFLITE_MODE "stub")

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" AND EXISTS ${TFLITE_X64_DIR}/libtensorflow-lite.a)
  set(WHISTLE_DETECTOR_TFLITE_MODE "bundled")
elseif(TFLITE_LIBRARY)
  set(WHISTLE_DETECTOR_TFLITE_MODE "system")
else()
  find_library(TFLITE_LIBRARY_AUTO
    NAMES tensorflowlite_c tensorflow-lite tensorflowlite
  )
  if(TFLITE_LIBRARY_AUTO)
    set(TFLITE_LIBRARY ${TFLITE_LIBRARY_AUTO})
    set(WHISTLE_DETECTOR_TFLITE_MODE "system")
  endif()
endif()

if(WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "bundled")
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

  set_target_properties(tensorflow-lite PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libtensorflow-lite.a)
  set_target_properties(ruy PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libruy.a)
  set_target_properties(farmhash PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libfarmhash.a)
  set_target_properties(flatbuffers PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libflatbuffers.a)
  set_target_properties(fft2d_fftsg PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libfft2d_fftsg.a)
  set_target_properties(fft2d_fftsg2d PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libfft2d_fftsg2d.a)
  set_target_properties(xnnpack PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libXNNPACK.a)
  set_target_properties(cpuinfo PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libcpuinfo.a)
  set_target_properties(clog PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libclog.a)
  set_target_properties(pthreadpool PROPERTIES IMPORTED_LOCATION ${TFLITE_X64_DIR}/libpthreadpool.a)
  set_target_properties(tensorflow-lite PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${TFLITE_THIRDPARTY_DIR})
elseif(WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "system")
  message(STATUS "whistle_detector using system TensorFlow Lite library: ${TFLITE_LIBRARY}")
elseif(NOT WHISTLE_DETECTOR_ALLOW_STUB_FALLBACK)
  message(FATAL_ERROR "No TensorFlow Lite library available. Set TFLITE_LIBRARY or provide bundled libraries.")
else()
  message(WARNING "No TensorFlow Lite library available; whistle detection will be built as a disabled stub.")
endif()

add_library(${PROJECT_NAME}_core)

if(WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "bundled" OR WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "system")
  target_sources(${PROJECT_NAME}_core PRIVATE
    src/alsa_whistle_detector.cpp
    src/tflite_executer.cpp
    src/tflite_whistle_detection.cpp
  )
else()
  target_sources(${PROJECT_NAME}_core PRIVATE
    src/alsa_whistle_detector_stub.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC WHISTLE_DETECTOR_TFLITE_UNAVAILABLE)
endif()

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TFLITE_THIRDPARTY_DIR}
  ${ALSA_INCLUDE_DIR}
)

if(FFTW3_INCLUDE_DIR AND FFTW3F_LIBRARY)
  target_include_directories(${PROJECT_NAME}_core PUBLIC ${FFTW3_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME}_core ${FFTW3F_LIBRARY})
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC WHISTLE_DETECTOR_USE_FFTW)
else()
  message(WARNING "FFTW headers or library not found; falling back to the internal DFT implementation.")
endif()

if(TFLITE_INCLUDE_DIR)
  target_include_directories(${PROJECT_NAME}_core PUBLIC ${TFLITE_INCLUDE_DIR})
endif()

target_link_libraries(${PROJECT_NAME}_core
  Boost::filesystem
  ${ALSA_LIBRARY}
  ${CMAKE_DL_LIBS}
  m
  pthread
  rt
)

if(WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "bundled")
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
  )
endif()

if((WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "bundled" OR WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "system") AND WHISTLE_DETECTOR_ENABLE_XNNPACK)
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC WHISTLE_DETECTOR_ENABLE_XNNPACK)
endif()

if(WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "system")
  target_link_libraries(${PROJECT_NAME}_core ${TFLITE_LIBRARY})
endif()

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

if(BUILD_TESTING AND (WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "bundled" OR WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "system"))
  ament_add_gtest(test_whistle_detection
    test/test_whistle_detection.cpp
  )
  target_link_libraries(test_whistle_detection ${PROJECT_NAME}_core)
  target_compile_features(test_whistle_detection PUBLIC cxx_std_17)
  target_compile_definitions(test_whistle_detection PRIVATE
    WHISTLE_DETECTOR_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
  )
endif()

ament_package()

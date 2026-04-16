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
set(TFLITE_BUNDLED_ARCH_DIR "")
set(TFLITE_BUNDLED_ARCH_NAME "")
set(TFLITE_REPO_LIBRARY "")
set(TFLITE_REPO_INCLUDE_DIR "")
set(TFLITE_BUNDLED_REQUIRED_LIBS
  libtensorflow-lite.a
  libruy.a
  libfarmhash.a
  libflatbuffers.a
  libfft2d_fftsg.a
  libfft2d_fftsg2d.a
)
set(TFLITE_BUNDLED_XNNPACK_LIBS
  libXNNPACK.a
  libcpuinfo.a
  libclog.a
  libpthreadpool.a
)
set(TFLITE_LIBRARY "" CACHE FILEPATH "Path to a TensorFlow Lite library for non-bundled builds")
set(TFLITE_INCLUDE_DIR "" CACHE PATH "Additional TensorFlow Lite include path for non-bundled builds")
set(WHISTLE_DETECTOR_TFLITE_MODE_OVERRIDE "auto" CACHE STRING "Override TensorFlow Lite backend selection: auto, bundled, system, or stub")
set_property(CACHE WHISTLE_DETECTOR_TFLITE_MODE_OVERRIDE PROPERTY STRINGS auto bundled system stub)
option(WHISTLE_DETECTOR_ALLOW_STUB_FALLBACK "Build a disabled stub if no TensorFlow Lite library is available" ON)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
  option(WHISTLE_DETECTOR_ENABLE_XNNPACK "Enable the TensorFlow Lite XNNPACK delegate" ON)
else()
  option(WHISTLE_DETECTOR_ENABLE_XNNPACK "Enable the TensorFlow Lite XNNPACK delegate" OFF)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
  set(TFLITE_BUNDLED_ARCH_NAME x64)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(TFLITE_BUNDLED_ARCH_NAME aarch64)
endif()

if(TFLITE_BUNDLED_ARCH_NAME)
  set(TFLITE_BUNDLED_ARCH_DIR ${TFLITE_THIRDPARTY_DIR}/${TFLITE_BUNDLED_ARCH_NAME})
  if(EXISTS ${TFLITE_BUNDLED_ARCH_DIR}/libtensorflowlite_c.so)
    set(TFLITE_REPO_LIBRARY ${TFLITE_BUNDLED_ARCH_DIR}/libtensorflowlite_c.so)
  elseif(EXISTS ${TFLITE_BUNDLED_ARCH_DIR}/libtensorflowlite.so)
    set(TFLITE_REPO_LIBRARY ${TFLITE_BUNDLED_ARCH_DIR}/libtensorflowlite.so)
  endif()
endif()

if(EXISTS ${TFLITE_THIRDPARTY_DIR}/include)
  set(TFLITE_REPO_INCLUDE_DIR ${TFLITE_THIRDPARTY_DIR}/include)
elseif(EXISTS ${TFLITE_THIRDPARTY_DIR}/tensorflow/lite/c/c_api.h OR EXISTS ${TFLITE_THIRDPARTY_DIR}/tflite_c_api.h)
  set(TFLITE_REPO_INCLUDE_DIR ${TFLITE_THIRDPARTY_DIR})
endif()

set(TFLITE_BUNDLED_COMPLETE FALSE)
set(_tflite_missing_bundled_libs "")
if(TFLITE_BUNDLED_ARCH_DIR)
  foreach(_bundled_lib IN LISTS TFLITE_BUNDLED_REQUIRED_LIBS)
    if(NOT EXISTS ${TFLITE_BUNDLED_ARCH_DIR}/${_bundled_lib})
      list(APPEND _tflite_missing_bundled_libs ${_bundled_lib})
    endif()
  endforeach()

  if(WHISTLE_DETECTOR_ENABLE_XNNPACK)
    foreach(_bundled_lib IN LISTS TFLITE_BUNDLED_XNNPACK_LIBS)
      if(NOT EXISTS ${TFLITE_BUNDLED_ARCH_DIR}/${_bundled_lib})
        list(APPEND _tflite_missing_bundled_libs ${_bundled_lib})
      endif()
    endforeach()
  endif()

  if(NOT _tflite_missing_bundled_libs)
    set(TFLITE_BUNDLED_COMPLETE TRUE)
  endif()
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

if(WHISTLE_DETECTOR_TFLITE_MODE_OVERRIDE STREQUAL "bundled")
  set(WHISTLE_DETECTOR_TFLITE_MODE "bundled")
elseif(WHISTLE_DETECTOR_TFLITE_MODE_OVERRIDE STREQUAL "system")
  set(WHISTLE_DETECTOR_TFLITE_MODE "system")
elseif(WHISTLE_DETECTOR_TFLITE_MODE_OVERRIDE STREQUAL "stub")
  set(WHISTLE_DETECTOR_TFLITE_MODE "stub")
elseif(NOT WHISTLE_DETECTOR_TFLITE_MODE_OVERRIDE STREQUAL "auto")
  message(FATAL_ERROR "Invalid WHISTLE_DETECTOR_TFLITE_MODE_OVERRIDE='${WHISTLE_DETECTOR_TFLITE_MODE_OVERRIDE}'. Expected auto, bundled, system, or stub.")
elseif(TFLITE_BUNDLED_COMPLETE)
  set(WHISTLE_DETECTOR_TFLITE_MODE "bundled")
elseif(TFLITE_REPO_LIBRARY)
  set(TFLITE_LIBRARY ${TFLITE_REPO_LIBRARY})
  if(NOT TFLITE_INCLUDE_DIR AND TFLITE_REPO_INCLUDE_DIR)
    set(TFLITE_INCLUDE_DIR ${TFLITE_REPO_INCLUDE_DIR})
  endif()
  set(WHISTLE_DETECTOR_TFLITE_MODE "system")
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

  set_target_properties(tensorflow-lite PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libtensorflow-lite.a)
  set_target_properties(ruy PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libruy.a)
  set_target_properties(farmhash PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libfarmhash.a)
  set_target_properties(flatbuffers PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libflatbuffers.a)
  set_target_properties(fft2d_fftsg PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libfft2d_fftsg.a)
  set_target_properties(fft2d_fftsg2d PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libfft2d_fftsg2d.a)
  set_target_properties(tensorflow-lite PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${TFLITE_THIRDPARTY_DIR})

  if(WHISTLE_DETECTOR_ENABLE_XNNPACK)
    add_library(xnnpack STATIC IMPORTED)
    add_library(cpuinfo STATIC IMPORTED)
    add_library(clog STATIC IMPORTED)
    add_library(pthreadpool STATIC IMPORTED)
    set_target_properties(xnnpack PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libXNNPACK.a)
    set_target_properties(cpuinfo PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libcpuinfo.a)
    set_target_properties(clog PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libclog.a)
    set_target_properties(pthreadpool PROPERTIES IMPORTED_LOCATION ${TFLITE_BUNDLED_ARCH_DIR}/libpthreadpool.a)
  endif()

  message(STATUS "whistle_detector using bundled TensorFlow Lite libraries from ${TFLITE_BUNDLED_ARCH_DIR}")
elseif(WHISTLE_DETECTOR_TFLITE_MODE STREQUAL "system")
  if(TFLITE_LIBRARY STREQUAL TFLITE_REPO_LIBRARY)
    message(STATUS "whistle_detector using repo TensorFlow Lite library: ${TFLITE_LIBRARY}")
  else()
    message(STATUS "whistle_detector using system TensorFlow Lite library: ${TFLITE_LIBRARY}")
  endif()
elseif(NOT WHISTLE_DETECTOR_ALLOW_STUB_FALLBACK)
  message(FATAL_ERROR "No TensorFlow Lite library available. Set TFLITE_LIBRARY or provide bundled libraries.")
else()
  if(TFLITE_BUNDLED_ARCH_DIR AND _tflite_missing_bundled_libs)
    list(JOIN _tflite_missing_bundled_libs ", " _tflite_missing_bundled_libs_message)
    message(WARNING "Bundled TensorFlow Lite directory '${TFLITE_BUNDLED_ARCH_DIR}' is incomplete; missing: ${_tflite_missing_bundled_libs_message}")
  endif()
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
  )
  if(WHISTLE_DETECTOR_ENABLE_XNNPACK)
    target_link_libraries(${PROJECT_NAME}_core
      xnnpack
      cpuinfo
      clog
      pthreadpool
    )
  endif()
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

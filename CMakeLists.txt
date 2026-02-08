cmake_minimum_required(VERSION 3.22)

project(gtsamGNSS
    VERSION 0.1.0
    DESCRIPTION "Standalone GNSS/robust factor extensions for modern GTSAM"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GTSAMGNSS_BUILD_TESTS "Build gtsamGNSS tests" ON)
option(GTSAMGNSS_ENABLE_WERROR "Treat warnings as errors" OFF)
option(GTSAMGNSS_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(GTSAMGNSS_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(GTSAMGNSS_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(GTSAMGNSS_ENABLE_LTO "Enable Link Time Optimization/IPO when supported" OFF)

if(GTSAMGNSS_ENABLE_TSAN AND (GTSAMGNSS_ENABLE_ASAN OR GTSAMGNSS_ENABLE_UBSAN))
  message(FATAL_ERROR "TSAN cannot be combined with ASAN/UBSAN in this project.")
endif()

set(_GTSAM_HINTS
    "$ENV{GTSAM_DIR}"
    "/opt/homebrew/lib/cmake/GTSAM"
    "/usr/local/lib/cmake/GTSAM"
    "/usr/lib/cmake/GTSAM")
find_package(Eigen3 CONFIG QUIET)
if(NOT TARGET Eigen3::Eigen)
  set(_GTSAMGNSS_EIGEN_HINTS "")
  if(DEFINED GTSAM_DIR)
    list(APPEND _GTSAMGNSS_EIGEN_HINTS
      "${GTSAM_DIR}/../eigen3-src"
      "${GTSAM_DIR}/../../eigen3-src")
  endif()
  find_path(GTSAMGNSS_EIGEN_INCLUDE_DIR Eigen/Core
      HINTS
      ${_GTSAMGNSS_EIGEN_HINTS}
      "/opt/homebrew/include/eigen3"
      "/usr/local/include/eigen3"
      "/usr/include/eigen3")
  if(GTSAMGNSS_EIGEN_INCLUDE_DIR)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE "${GTSAMGNSS_EIGEN_INCLUDE_DIR}")
  endif()
endif()
find_package(GTSAM CONFIG QUIET HINTS ${_GTSAM_HINTS})
if(NOT GTSAM_FOUND)
  message(FATAL_ERROR
    "GTSAM was not found. Set GTSAM_DIR (or CMAKE_PREFIX_PATH) to the directory containing GTSAMConfig.cmake. "
    "Examples: /opt/homebrew/lib/cmake/GTSAM (Apple Silicon), /usr/local/lib/cmake/GTSAM (Intel macOS/Linux).")
endif()

file(GLOB GTSAMGNSS_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gtsam/gnssNavigation/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gtsam/robustModels/*.h")

set(GTSAMGNSS_SOURCES
    src/gnssNavigation/FolderUtils.cpp
    src/gnssNavigation/GNSSDCSFactor.cpp
    src/gnssNavigation/GNSSFactor.cpp
    src/gnssNavigation/GNSSMultiModalFactor.cpp
    src/gnssNavigation/GnssData.cpp
    src/gnssNavigation/GnssPostfit.cpp
    src/gnssNavigation/GnssTools.cpp
    src/gnssNavigation/nonBiasStates.cpp
    src/robustModels/GNSSMaxMix.cpp
    src/robustModels/GNSSSwitch.cpp
    src/robustModels/PhaseSwitchFactor.cpp
    src/robustModels/PseudorangeMaxMix.cpp
    src/robustModels/PseudorangeSwitchFactor.cpp
    src/robustModels/switchPairLinear.cpp
    src/robustModels/switchVariableLinear.cpp
    src/robustModels/betweenFactorSwitchable.cpp)

add_library(gtsamGNSS STATIC ${GTSAMGNSS_SOURCES} ${GTSAMGNSS_HEADERS})
add_library(gtsamGNSS::gtsamGNSS ALIAS gtsamGNSS)

target_include_directories(gtsamGNSS
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(TARGET GTSAM::gtsam)
  target_link_libraries(gtsamGNSS PUBLIC GTSAM::gtsam)
else()
  target_link_libraries(gtsamGNSS PUBLIC gtsam)
endif()

set(_GTSAMGNSS_WARNING_FLAGS
    $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:-Wall;-Wextra;-Wpedantic>)
if(GTSAMGNSS_ENABLE_WERROR)
  list(APPEND _GTSAMGNSS_WARNING_FLAGS
      $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:-Werror>)
endif()

set(_GTSAMGNSS_SANITIZER_FLAGS "")
if(GTSAMGNSS_ENABLE_ASAN)
  list(APPEND _GTSAMGNSS_SANITIZER_FLAGS -fsanitize=address)
endif()
if(GTSAMGNSS_ENABLE_UBSAN)
  list(APPEND _GTSAMGNSS_SANITIZER_FLAGS -fsanitize=undefined)
endif()
if(GTSAMGNSS_ENABLE_TSAN)
  list(APPEND _GTSAMGNSS_SANITIZER_FLAGS -fsanitize=thread)
endif()

function(gtsamgnss_apply_quality_flags target_name)
  target_compile_options(${target_name} PRIVATE ${_GTSAMGNSS_WARNING_FLAGS})
  if(_GTSAMGNSS_SANITIZER_FLAGS)
    foreach(_san_flag IN LISTS _GTSAMGNSS_SANITIZER_FLAGS)
      target_compile_options(${target_name} PRIVATE
          $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:${_san_flag}>)
      target_link_options(${target_name} PRIVATE
          $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:${_san_flag}>)
    endforeach()
  endif()
endfunction()

gtsamgnss_apply_quality_flags(gtsamGNSS)

if(GTSAMGNSS_ENABLE_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT gtsamgnss_ipo_supported OUTPUT gtsamgnss_ipo_error)
  if(gtsamgnss_ipo_supported)
    set_property(TARGET gtsamGNSS PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(WARNING "LTO requested but not supported: ${gtsamgnss_ipo_error}")
  endif()
endif()

if(GTSAMGNSS_BUILD_TESTS)
  enable_testing()
  add_executable(gtsamgnss_factor_math_test test/test_factor_math.cpp)
  if(TARGET GTSAM::gtsam)
    target_link_libraries(gtsamgnss_factor_math_test PRIVATE gtsamGNSS::gtsamGNSS GTSAM::gtsam)
  else()
    target_link_libraries(gtsamgnss_factor_math_test PRIVATE gtsamGNSS::gtsamGNSS gtsam)
  endif()
  gtsamgnss_apply_quality_flags(gtsamgnss_factor_math_test)
  if(GTSAMGNSS_ENABLE_LTO AND gtsamgnss_ipo_supported)
    set_property(TARGET gtsamgnss_factor_math_test PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
  add_test(NAME factor_math_sanity COMMAND gtsamgnss_factor_math_test)
endif()

include(GNUInstallDirs)
install(TARGETS gtsamGNSS
    EXPORT gtsamGNSSTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT gtsamGNSSTargets
    FILE gtsamGNSSConfig.cmake
    NAMESPACE gtsamGNSS::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gtsamGNSS)

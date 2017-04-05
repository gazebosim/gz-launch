include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

# Detect the architecture
include (${project_cmake_dir}/TargetArch.cmake)
target_architecture(ARCH)
message(STATUS "Building for arch: ${ARCH}")

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/ 
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

########################################
# 1. can not use BUILD_TYPE_PROFILE is defined after include this module
# 2. TODO: TOUPPER is a hack until we fix the build system to support
# standard build names
if (CMAKE_BUILD_TYPE)
  string(TOUPPER ${CMAKE_BUILD_TYPE} TMP_CMAKE_BUILD_TYPE)
  if ("${TMP_CMAKE_BUILD_TYPE}" STREQUAL "PROFILE")
    include (${project_cmake_dir}/FindGooglePerfTools.cmake)
    if (GOOGLE_PERFTOOLS_FOUND)
      message(STATUS "Include google-perftools")
    else()
      BUILD_ERROR("Need google/heap-profiler.h (libgoogle-perftools-dev) "
      "tools to compile in Profile mode")
    endif()
  endif()
endif()

########################################
# Find Ignition Common
find_package(ignition-common0 QUIET)
if (NOT ignition-common0_FOUND)
  BUILD_ERROR ("Missing: Ignition Common (libignition-common0-dev)")
else()
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-COMMON_CXX_FLAGS}")
  include_directories(${IGNITION-COMMON_INCLUDE_DIRS})
  link_directories(${IGNITION-COMMON_LIBRARY_DIRS})
endif()

#################################################
# Find tinyxml2. Only debian distributions package tinyxml with a pkg-config
# Use pkg_check_modules and fallback to manual detection
# (needed, at least, for MacOS)

# Use system installation on UNIX and Apple, and internal copy on Windows
if (UNIX OR APPLE)
  message (STATUS "Using system tinyxml2.")
  set (USE_EXTERNAL_TINYXML2 True)
elseif(WIN32)
  message (STATUS "Using internal tinyxml2.")
  set (USE_EXTERNAL_TINYXML2 False)
else()
  message (STATUS "Unknown platform, unable to configure tinyxml2.")
  BUILD_ERROR("Unknown platform")
endif()

if (USE_EXTERNAL_TINYXML2)
  pkg_check_modules(tinyxml2 tinyxml2)
  if (NOT tinyxml2_FOUND)
      find_path (tinyxml2_INCLUDE_DIRS tinyxml2.h ${tinyxml2_INCLUDE_DIRS} ENV CPATH)
      find_library(tinyxml2_LIBRARIES NAMES tinyxml2)
      set (tinyxml2_FAIL False)
      if (NOT tinyxml2_INCLUDE_DIRS)
        message (STATUS "Looking for tinyxml2 headers - not found")
        set (tinyxml2_FAIL True)
      endif()
      if (NOT tinyxml2_LIBRARIES)
        message (STATUS "Looking for tinyxml2 library - not found")
        set (tinyxml2_FAIL True)
      endif()
      if (NOT tinyxml2_LIBRARY_DIRS)
        message (STATUS "Looking for tinyxml2 library dirs - not found")
        set (tinyxml2_FAIL True)
      endif()
  endif()

  if (tinyxml2_FAIL)
    message (STATUS "Looking for tinyxml2.h - not found")
    BUILD_ERROR("Missing: tinyxml2")
  endif()
else()
  # Needed in WIN32 since in UNIX the flag is added in the code installed
  message (STATUS "Skipping search for tinyxml2")
  set (tinyxml2_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/deps/tinyxml2")
  set (tinyxml2_LIBRARIES "")
  set (tinyxml2_LIBRARY_DIRS "")
endif()

########################################
# GFlags
find_library(gflags_LIBRARIES NAMES gflags)
find_path(gflags_INCLUDE_DIRS gflags/gflags.h ENV CPATH)
if (NOT gflags_LIBRARIES OR NOT gflags_INCLUDE_DIRS)
  BUILD_ERROR("GFlags package is missing.")
endif()

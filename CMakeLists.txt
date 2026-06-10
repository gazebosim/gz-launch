cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(gz-launch VERSION 9.0.1)

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake 5 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

gz_configure_project(VERSION_SUFFIX)

#============================================================================
# Set project-specific options
#============================================================================

set(GZ_LAUNCH_PLUGIN_RELATIVE_INSTALL_PATH "${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/")

#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find Tinyxml2
gz_find_package(TINYXML2 REQUIRED PRIVATE PRETTY tinyxml2)

#--------------------------------------
# Find gz-common
gz_find_package(gz-common VERSION 7 REQUIRED PRIVATE)

#--------------------------------------
# Find gz-utils
gz_find_package(gz-utils VERSION 4 REQUIRED COMPONENTS cli)

#--------------------------------------
# Find gz-plugin
gz_find_package(gz-plugin VERSION 4 REQUIRED_BY launch COMPONENTS loader register)

#--------------------------------------
# Find if gz command is available
find_program(GZ_TOOLS_PROGRAM gz)
if (GZ_TOOLS_PROGRAM)
  message (STATUS "Searching for gz program - found. CLI tests can be built.")
else()
  message (STATUS "Searching for gz program - not found. CLI tests are skipped.")
endif()
set(GZ_TOOLS_VER 2)

#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport VERSION 15 REQUIRED)

#--------------------------------------
# Find gz-msgs
gz_find_package(gz-msgs VERSION 12 REQUIRED)

#--------------------------------------
# Find gz-math
gz_find_package(gz-math VERSION 9 REQUIRED)

#--------------------------------------
# Find gz-gui
gz_find_package(gz-gui VERSION 10 REQUIRED)

#--------------------------------------
# Find gz-sim
gz_find_package(gz-sim VERSION 10 REQUIRED PRIVATE COMPONENTS gui)

#--------------------------------------
# Find gz-fuel_tools
gz_find_package(gz-fuel_tools VERSION 11 REQUIRED PRIVATE)
set(GZ_FUEL_TOOLS_VER ${gz-fuel_tools_VERSION_MAJOR})

gz_pkg_check_modules(websockets libwebsockets)
if (NOT websockets_FOUND)
  gz_build_warning("Unable to find libwebsockets. The websocket_server plugin will not be built.")
endif()

#--------------------------------------
# Find prerequistes for backward
include(src/vendor/BackwardConfig.cmake)

#============================================================================
# Add plugins
#============================================================================
add_subdirectory(plugins)

#============================================================================
# Configure the build
#============================================================================
gz_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# install example .gzlaunch files
#============================================================================
add_subdirectory(examples)

#============================================================================
# Create package information
#============================================================================
gz_create_packages()

#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

gz_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md"
  TAGFILES
   "${GZ-MATH_DOXYGEN_TAGFILE} = ${GZ-MATH_API_URL}"
   "${GZ-MSGS_DOXYGEN_TAGFILE} = ${GZ-MSGS_API_URL}"
   "${GZ-PLUGIN_DOXYGEN_TAGFILE} = ${GZ-PLUGIN_API_URL}"
   "${GZ-TRANSPORT_DOXYGEN_TAGFILE} = ${GZ-TRANSPORT_API_URL}"
   "${GZ-COMMON_DOXYGEN_TAGFILE} = ${GZ-COMMON_API_URL}"
   "${GZ-SIM_DOXYGEN_TAGFILE} = ${GZ-SIM_API_URL}"
   "${GZ-GUI_DOXYGEN_TAGFILE} = ${GZ-GUI_API_URL}"
)

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-launch2 VERSION 2.0.1)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake2 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

#============================================================================
# Configure the project
#============================================================================
ign_configure_project()

# Create cppcheck suppression list file.
configure_file(tools/cppcheck.suppress.in ${CMAKE_BINARY_DIR}/cppcheck.suppress)

#============================================================================
# Set project-specific options
#============================================================================

set(IGN_LAUNCH_PLUGIN_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/")

#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find Tinyxml2
ign_find_package(TINYXML2 REQUIRED PRIVATE PRETTY tinyxml2)

#--------------------------------------
# Find ignition-common
ign_find_package(ignition-common3 REQUIRED PRIVATE)
set(IGN_COMMON_MAJOR_VER ${ignition-common3_VERSION_MAJOR})

#--------------------------------------
# Find ignition-plugin
ign_find_package(ignition-plugin1 REQUIRED_BY launch COMPONENTS loader register)
set(IGN_PLUGIN_MAJOR_VER ${ignition-plugin1_VERSION_MAJOR})

#--------------------------------------
# Find ignition-tools
ign_find_package(ignition-tools REQUIRED)

#--------------------------------------
# Find ignition-transport
ign_find_package(ignition-transport8 REQUIRED)
set(IGN_TRANSPORT_MAJOR_VER ${ignition-transport8_VERSION_MAJOR})

#--------------------------------------
# Find ignition-msgs
ign_find_package(ignition-msgs5 REQUIRED)
set(IGN_MSGS_MAJOR_VER ${ignition-msgs5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math6 REQUIRED)
set(IGN_MATH_MAJOR_VER ${ignition-math6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-gui
ign_find_package(ignition-gui3 REQUIRED)
set(IGN_GUI_MAJOR_VER ${ignition-gui3_VERSION_MAJOR})
ign_find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
  PKGCONFIG "Qt5Core Qt5Quick Qt5QuickControls2")

#--------------------------------------
# Find ignition-gazebo
ign_find_package(ignition-gazebo3 REQUIRED PRIVATE COMPONENTS gui)
set(IGN_GAZEBO_MAJOR_VER ${ignition-gazebo3_VERSION_MAJOR})

ign_pkg_check_modules(websockets libwebsockets)
if (NOT websockets_FOUND)
  ign_build_warning("Unable to find libwebsockets. The websocket_server plugin will not be built.")
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
ign_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# install example .ign files
#============================================================================
add_subdirectory(examples)

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

ign_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md"
  TAGFILES
   "${IGNITION-MATH_DOXYGEN_TAGFILE} = ${IGNITION-MATH_API_URL}"
   "${IGNITION-MSGS_DOXYGEN_TAGFILE} = ${IGNITION-MSGS_API_URL}"
   "${IGNITION-PHYSICS_DOXYGEN_TAGFILE} = ${IGNITION-PHYSICS_API_URL}"
   "${IGNITION-PLUGIN_DOXYGEN_TAGFILE} = ${IGNITION-PLUGIN_API_URL}"
   "${IGNITION-TRANSPORT_DOXYGEN_TAGFILE} = ${IGNITION-TRANSPORT_API_URL}"
   "${IGNITION-SENSORS_DOXYGEN_TAGFILE} = ${IGNITION-SENSORS_API_URL}"
   "${IGNITION-COMMON_DOXYGEN_TAGFILE} = ${IGNITION-COMMON_API_URL}"
   "${IGNITION-GAZEBO_DOXYGEN_TAGFILE} = ${IGNITION-GAZEBO_API_URL}"
   "${IGNITION-GUI_DOXYGEN_TAGFILE} = ${IGNITION-GUI_API_URL}"
)

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-launch0 VERSION 0.1.0)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake2 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

#============================================================================
# Configure the project
#============================================================================
ign_configure_project()

#============================================================================
# Set project-specific options
#============================================================================

# No project-specific options yet for ign-plugin

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
ign_find_package(ignition-transport6 REQUIRED)
set(IGN_TRANSPORT_MAJOR_VER ${ignition-transport6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-msgs
ign_find_package(ignition-msgs3 REQUIRED)
set(IGN_MSGS_MAJOR_VER ${ignition-msgs3_VERSION_MAJOR})

#--------------------------------------
# Find ignition-gui
ign_find_package(ignition-gui1 REQUIRED)
set(IGN_GUI_MAJOR_VER ${ignition-gui1_VERSION_MAJOR})
ign_find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
  PKGCONFIG "Qt5Core Qt5Quick Qt5QuickControls2")

#--------------------------------------
# Find ignition-gazebo
ign_find_package(ignition-gazebo0 REQUIRED PRIVATE COMPONENTS gui)
set(IGN_GAZEBO_MAJOR_VER ${ignition-gazebo0_VERSION_MAJOR})

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# ign command line support
#============================================================================
add_subdirectory(plugins)

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

#============================================================================
# Configure documentation
#============================================================================
ign_create_docs()

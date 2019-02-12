cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-launch VERSION 0.1.0)

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
# Find ignition-tools
ign_find_package(ignition-tools REQUIRED PRIVATE)

################################################
# Find Ruby, which is used to produce xml schemas and parse ERB files.
find_package(Ruby)
if (RUBY_FOUND)
  message(STATUS "Looking for ruby - found version ${RUBY_VERSION}")
else()
  message(STATUS "Looking for ruby - not found")
  BUILD_ERROR("Couldn't find Ruby")
endif()

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# ign command line support
#============================================================================
add_subdirectory(conf)
add_subdirectory(plugins)

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

#============================================================================
# Configure documentation
#============================================================================
ign_create_docs()

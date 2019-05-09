# add_subdirectory(plugins)

set (sources
  ign.cc
  Manager.cc
)

set (gtest_sources
  Manager_TEST.cc
  ign_TEST.cc
)

# Create the library target.
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

# Link the libraries that we always need.
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ignition-plugin${IGN_PLUGIN_MAJOR_VER}::loader
    ignition-plugin${IGN_PLUGIN_MAJOR_VER}::register
    TINYXML2::TINYXML2
  PRIVATE
    ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
)

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources}
                EXCLUDE_PROJECT_LIB
                LIB_DEPS
                  ${PROJECT_LIBRARY_TARGET_NAME}
                  ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
)

#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdtransport0.rb
set(cmd_script_generated_test "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured_test "${cmd_script_generated_test}.configured")

# Set the library_location variable to the full path of the library file within
# the build directory.
set(library_location "$<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>")

configure_file(
  "cmd${IGN_DESIGNATION}.rb.in"
  "${cmd_script_configured_test}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated_test}"
  INPUT  "${cmd_script_configured_test}")

# Used only for internal testing.
set(ign_library_path "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}")

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: transport0.yaml
configure_file(
  "${IGN_DESIGNATION}.yaml.in"
    "${CMAKE_BINARY_DIR}/test/conf/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

#===============================================================================
# Used for the installed version.
# Generate the ruby script that gets installed.
# Note that the major version of the library is included in the name.
# Ex: cmdtransport0.rb
set(cmd_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured "${cmd_script_generated}.configured")

# Set the library_location variable to the relative path to the library file
# within the install directory structure.
set(library_location "../../../${CMAKE_INSTALL_LIBDIR}/$<TARGET_FILE_NAME:${PROJECT_LIBRARY_TARGET_NAME}>")

configure_file(
  "cmd${IGN_DESIGNATION}.rb.in"
  "${cmd_script_configured}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated}"
  INPUT  "${cmd_script_configured}")

# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_script_generated} DESTINATION lib/ruby/ignition)

# Used for the installed version.
set(ign_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}")

# Generate the configuration file that is installed.
# Note that the major version of the library is included in the name.
# Ex: transport0.yaml
configure_file(
  "${IGN_DESIGNATION}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/)

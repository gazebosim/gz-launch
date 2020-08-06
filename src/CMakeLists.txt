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
    ignition-fuel_tools${IGN_FUEL_TOOLS_VER}::ignition-fuel_tools${IGN_FUEL_TOOLS_VER}
    TINYXML2::TINYXML2
  PRIVATE
    ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
    ${BACKWARD_LIBRARIES}
)

target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE ${BACKWARD_DEFINITIONS})

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources}
                EXCLUDE_PROJECT_LIB
                LIB_DEPS
                  ${PROJECT_LIBRARY_TARGET_NAME}
                  ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
)

# Generate a the ruby script.
# Note that the major version of the library is included in the name.
# Ex: cmdlaunch0.rb
if (APPLE)
  set(IGN_LIBRARY_NAME lib${PROJECT_LIBRARY_TARGET_NAME}.dylib)
else()
  set(IGN_LIBRARY_NAME lib${PROJECT_LIBRARY_TARGET_NAME}.so)
endif()

configure_file(
  "cmdlaunch.rb.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmdlaunch${PROJECT_VERSION_MAJOR}.rb" @ONLY)


# Install the ruby command line library in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmdlaunch${PROJECT_VERSION_MAJOR}.rb DESTINATION lib/ruby/ignition)

set(ign_library_path
  "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmdlaunch${PROJECT_VERSION_MAJOR}")

# Generate a configuration file.
# Note that the major version of the library is included in the name.
# Ex: fuel0.yaml
configure_file(
  "launch.yaml.in"
    "${CMAKE_CURRENT_BINARY_DIR}/launch${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/launch${PROJECT_VERSION_MAJOR}.yaml DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/)

#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdlaunch0.rb
set(cmd_script_generated_test "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmdlaunch${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured_test "${cmd_script_generated_test}.configured")

# Set the library_location variable to the full path of the library file within
# the build directory.
set(IGN_LIBRARY_NAME "$<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>")

configure_file(
  "cmdlaunch.rb.in"
  "${cmd_script_configured_test}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated_test}"
  INPUT  "${cmd_script_configured_test}")

# Used only for internal testing.
set(ign_library_path "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmdlaunch${PROJECT_VERSION_MAJOR}")

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: launch0.yaml
configure_file(
  "launch.yaml.in"
    "${CMAKE_BINARY_DIR}/test/conf/launch${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

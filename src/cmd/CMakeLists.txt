# Generate a the ruby script.
# Note that the major version of the library is included in the name.
# Ex: cmdlaunch0.rb
if (APPLE)
  set(IGN_LIBRARY_NAME lib${PROJECT_NAME_LOWER}.dylib)
else()
  set(IGN_LIBRARY_NAME lib${PROJECT_NAME_LOWER}.so)
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

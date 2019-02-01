set(ign_library_path "${CMAKE_BINARY_DIR}/src/cmd/cmdlaunch${PROJECT_VERSION_MAJOR}")

set(ign_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmdlaunch${PROJECT_VERSION_MAJOR}")

# Generate a configuration file.
# Note that the major version of the library is included in the name.
# Ex: fuel0.yaml
configure_file(
  "launch.yaml.in"
    "${CMAKE_CURRENT_BINARY_DIR}/launch${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/launch${PROJECT_VERSION_MAJOR}.yaml DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/)

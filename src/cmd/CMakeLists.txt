# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
ign_get_libsources_and_unittests(sources gtest_sources)

# Disable tests that need CLI if ign-tools is not found
if (MSVC OR NOT IGN_TOOLS_PROGRAM)
  list(REMOVE_ITEM gtest_sources ign_TEST.cc)
endif()

add_library(ign STATIC ign.cc)
target_include_directories(ign PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ign PUBLIC
  ${PROJECT_LIBRARY_TARGET_NAME}
  ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
  ignition-utils${IGN_UTILS_VER}::core
)

set(launch_executable ign-launch)
add_executable(${launch_executable} launch_main.cc)
target_link_libraries(${launch_executable}
  ign
  ignition-utils${IGN_UTILS_VER}::cli
  ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
  ignition-utils${IGN_UTILS_VER}::core
)

install(
  TARGETS
  ${launch_executable}
  DESTINATION
  ${IGN_LIB_INSTALL_DIR}/ignition/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}/
)

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources}
  TEST_LIST test_list
  LIB_DEPS ${EXTRA_TEST_LIB_DEPS})

foreach(test ${test_list})
  target_link_libraries(${test} ign)

  # Inform each test of its output directory so it knows where to call the
  # auxiliary files from. Using a generator expression here is useful for
  # multi-configuration generators, like Visual Studio.
  target_compile_definitions(${test} PRIVATE
    "DETAIL_IGN_TRANSPORT_TEST_DIR=\"$<TARGET_FILE_DIR:${test}>\""
    "IGN_TEST_LIBRARY_PATH=\"$<TARGET_FILE_DIR:${PROJECT_LIBRARY_TARGET_NAME}>\"")

endforeach()

if (TARGET UNIT_ign_TEST)
  set_tests_properties(
    UNIT_ign_TEST
    PROPERTIES
    ENVIRONMENT
      "IGN_CONFIG_PATH=${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>"
  )

  # Running `ign launch` on macOS has problems when run with /usr/bin/ruby
  # due to System Integrity Protection (SIP). Try to find ruby from
  # homebrew as a workaround.
  if (APPLE)
    find_program(BREW_RUBY ruby HINTS /usr/local/opt/ruby/bin)
  endif()

  target_compile_definitions(UNIT_ign_TEST PRIVATE
      "BREW_RUBY=\"${BREW_RUBY} \"")

  target_compile_definitions(UNIT_ign_TEST PRIVATE
      "IGN_PATH=\"${HAVE_IGN_TOOLS}\"")
endif()

#===============================================================================
# Bash completion

# Tack version onto and install the bash completion script
configure_file(
  "launch.bash_completion.sh"
    "${CMAKE_CURRENT_BINARY_DIR}/launch${PROJECT_VERSION_MAJOR}.bash_completion.sh" @ONLY)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/launch${PROJECT_VERSION_MAJOR}.bash_completion.sh
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gz/gz${IGN_TOOLS_VER}.completion.d)

#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdlaunch5.rb

set(cmd_script_generated_test "${CMAKE_BINARY_DIR}/test/lib/$<CONFIG>/ruby/ignition/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured_test "${CMAKE_CURRENT_BINARY_DIR}/test_cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb.configured")

# Set the library_location variable to the full path of the library file within
# the build directory.
set(launch_exe_location "$<TARGET_FILE:${launch_executable}>")

configure_file(
  "cmd${IGN_DESIGNATION}.rb.in"
  "${cmd_script_configured_test}"
  @ONLY
)

file(GENERATE
  OUTPUT "${cmd_script_generated_test}"
  INPUT  "${cmd_script_configured_test}"
)

#===============================================================================
# Used for the installed version.
# Generate the ruby script that gets installed.
# Note that the major version of the library is included in the name.
# Ex: cmdlaunch5.rb
set(cmd_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured "${cmd_script_generated}.configured")

# Set the library_location variable to the relative path to the library file
# within the install directory structure.
set(launch_exe_location "../../../${CMAKE_INSTALL_LIBDIR}/ignition/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}/$<TARGET_FILE_NAME:${launch_executable}>")

configure_file(
  "cmd${IGN_DESIGNATION}.rb.in"
  "${cmd_script_configured}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated}"
  INPUT  "${cmd_script_configured}")

set(ign_library_path "${CMAKE_BINARY_DIR}/test/lib/$<CONFIG>/ruby/ignition/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}")

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: launch5.yaml
configure_file(
  "${IGN_DESIGNATION}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml.configured" @ONLY)

file(GENERATE
  OUTPUT "${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml.configured")

# Used for the installed version.
set(ign_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}")

# Generate the configuration file that is installed.
# Note that the major version of the library is included in the name.
# Ex: launch5.yaml
configure_file(
  "${IGN_DESIGNATION}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/)

# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_script_generated} DESTINATION lib/ruby/ignition)

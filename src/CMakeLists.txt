# add_subdirectory(plugins)

set (sources
  Manager.cc
)

set (gtest_sources
  Manager_TEST.cc
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
    ignition-math${IGN_MATH_MAJOR_VER}::ignition-math${IGN_MATH_MAJOR_VER}
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

# if(TARGET UNIT_ign_TEST)
#   # Running `ign launch` on macOS has problems when run with /usr/bin/ruby
#   # due to System Integrity Protection (SIP). Try to find ruby from
#   # homebrew as a workaround.
#   if (APPLE)
#     find_program(BREW_RUBY ruby HINTS /usr/local/opt/ruby/bin)
#   endif()
# 
#   target_compile_definitions(UNIT_ign_TEST PRIVATE
#       "BREW_RUBY=\"${BREW_RUBY} \"")
# 
#   target_compile_definitions(UNIT_ign_TEST PRIVATE
#       "IGN_PATH=\"${HAVE_IGN_TOOLS}\"")
# endif()

add_subdirectory(cmd)

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

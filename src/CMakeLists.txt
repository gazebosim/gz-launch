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
                  ignition-utils${IGN_UTILS_VER}::core
)

add_subdirectory(cmd)

set (sources
  ign.cc
  Manager.cc
)

set (gtest_sources
  Manager_TEST.cc
)

# Create the library target.
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

# Link the libraries that we always need.
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
    TINYXML2::TINYXML2
)

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources}
                LIB_DEPS
                  ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
                  TINYXML2::TINYXML2
)

# Command line support.
add_subdirectory(cmd)

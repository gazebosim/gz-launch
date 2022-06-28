# add_subdirectory(plugins)

set (sources
  Manager.cc
)

set (gtest_sources
  Manager_TEST.cc
)

# Create the library target.
gz_create_core_library(SOURCES ${sources})

# Link the libraries that we always need.
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    gz-plugin${GZ_PLUGIN_VER}::loader
    gz-plugin${GZ_PLUGIN_VER}::register
    TINYXML2::TINYXML2
  PRIVATE
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    ${BACKWARD_LIBRARIES}
)

target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE ${BACKWARD_DEFINITIONS})

# Build the unit tests.
gz_build_tests(TYPE UNIT SOURCES ${gtest_sources}
                EXCLUDE_PROJECT_LIB
                LIB_DEPS
                  ${PROJECT_LIBRARY_TARGET_NAME}
                  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

add_subdirectory(cmd)

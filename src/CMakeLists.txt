set (sources
  ign.cc
  Manager.cc
)

set (gtest_sources
  Manager_TEST.cc
)

# if (HAVE_IGN_TOOLS)
#   set (gtest_sources ${gtest_sources}
#     ign_TEST.cc
#   )
# endif()

include_directories(${RUBY_INCLUDE_DIRS})

# Create the library target.
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

# Link the libraries that we always need.
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    TINYXML2::TINYXML2
    ${RUBY_LIBRARY}
)

# ign_target_interface_include_directories(${PROJECT_LIBRARY_TARGET_NAME})

# Build the unit tests.
ign_build_tests(TYPE UNIT
                SOURCES ${gtest_sources}
                LIB_DEPS
                  ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
                  TINYXML2::TINYXML2
)

# Command line support.
add_subdirectory(cmd)

# if (UNIX)
#   ign_add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} ${source})
# else()
#   add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} STATIC ${source})
# endif()
# 
# target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
#   ${IGNITION-COMMON_LIBRARIES}
#   ${tinyxml2_LIBRARIES}
#   ${gflags_LIBRARIES}
# )
# 
# ign_add_executable(ignition ${source} ignition.cc)
# target_link_libraries(ignition
#   ${IGNITION-COMMON_LIBRARIES}
#   ${tinyxml2_LIBRARIES}
#   ${gflags_LIBRARIES})
# 
# if (UNIX)
#   target_link_libraries(ignition pthread rt)
# endif()
# 
# if ("${CMAKE_BUILD_TYPE}" STREQUAL "COVERAGE")
#   target_link_libraries(ignition gcov)
# endif()
# 
# ign_install_executable(ignition)

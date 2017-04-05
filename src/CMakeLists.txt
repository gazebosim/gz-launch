include_directories(SYSTEM
  ${gflags_INCLUDE_DIRS}
  ${tinyxml2_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_BINARY_DIR}/include
)

link_directories(
  ${tinyxml2_LIBRARY_DIRS}
  ${PROJECT_BINARY_DIR}/test
)

set (source
  Manager.cc
)

set (gtest_sources
  Manager_TEST.cc
)
ign_build_tests(${gtest_sources})

if (UNIX)
  ign_add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} ${source})
else()
  add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} STATIC ${source})
endif()

target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
  ${IGNITION-COMMON_LIBRARIES}
  ${tinyxml2_LIBRARIES}
  ${gflags_LIBRARIES}
)

ign_add_executable(ignition ${source} ignition.cc)
target_link_libraries(ignition
  ${IGNITION-COMMON_LIBRARIES}
  ${tinyxml2_LIBRARIES}
  ${gflags_LIBRARIES})

if (UNIX)
  target_link_libraries(ignition pthread rt)
endif()

if ("${CMAKE_BUILD_TYPE}" STREQUAL "COVERAGE")
  target_link_libraries(ignition gcov)
endif()

ign_install_executable(ignition)

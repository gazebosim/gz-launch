
add_library(bad_plugins SHARED bad_plugins.cc)

target_link_libraries(
  bad_plugins
  PRIVATE
  ignition-plugin${IGN_PLUGIN_MAJOR_VER}::core
  ${PROJECT_LIBRARY_TARGET_NAME}
)

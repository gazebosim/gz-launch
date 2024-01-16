ign_install_all_headers()

if (WIN32)
  configure_file (${CMAKE_CURRENT_SOURCE_DIR}/strings.h.in
                  ${CMAKE_BINARY_DIR}/include/strings.h)
endif()

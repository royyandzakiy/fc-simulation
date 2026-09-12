# cmake/package_manager.cmake
# One macro to select the dependency provider, driven by the PKG_MANAGER cache variable
# (defined in project_options.cmake). Call setup_package_manager() BEFORE project().
#
# Swap providers by setting PKG_MANAGER = vcpkg | conan | none via -D, project_options.local.cmake,
# the IDE's CMake cache editor, or a preset. Switching managers needs a fresh configure
# (a separate build dir), since the toolchain/provider is fixed at first configure.

macro(setup_package_manager)
  if(PKG_MANAGER STREQUAL "vcpkg")
    message(STATUS "[pkg] Package manager: vcpkg")
    include(cmake/config_vcpkg.cmake)
  elseif(PKG_MANAGER STREQUAL "conan")
    message(STATUS "[pkg] Package manager: Conan")
    include(cmake/config_conan.cmake)
  elseif(PKG_MANAGER STREQUAL "none")
    message(STATUS "[pkg] Package manager: none (system libraries / find_package only)")
  else()
    message(FATAL_ERROR "[pkg] Invalid PKG_MANAGER='${PKG_MANAGER}' (use: vcpkg | conan | none)")
  endif()
endmacro()

cmake_minimum_required(VERSION 3.28)

#=========================
# Project Options & Package Manager
#=========================
include(project_options.local.cmake OPTIONAL) # create & modify to change locally (only for you)
include(project_options.cmake) # modify to change for all developers

include(cmake/package_manager.cmake)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  # If this project is consumed via add_subdirectory(), the parent owns the provider/toolchain, hence
  # omit this setup_package_maanager(). But because before project(), PROJECT_IS_TOP_LEVEL isn't set yet
  # hence use the source-dir check
  set(PKG_MANAGER "conan")
  # set(PKG_MANAGER "vcpkg")
  
  setup_package_manager() # dispatches to vcpkg / conan / none per PKG_MANAGER
endif()

#=========================
# Project Configuration
#=========================
include(cmake/version.cmake) # reads version.txt -> PROJECT_VERSION_STRING (before project())
project(
  fc_simulation
  LANGUAGES CXX
  VERSION ${PROJECT_VERSION_STRING})

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#=========================
# Formatter, Linter, Profiler, Accelerator
#=========================
if(PROJECT_IS_TOP_LEVEL) # format & setup clangd
  include(cmake/format.cmake) # format / format-check / tidy-all targets
  include(cmake/clangd.cmake) # mirror compile_commands.json for clangd
endif()

include(cmake/compiler.cmake)            # warnings, strict, build-type guard, debug postfix
include(cmake/build_acceleration.cmake)  # fast linker (mold/lld) + ccache

include(cmake/sanitizers.cmake)          # ASan/TSan/MSan/UBSan flags + deploy_asan_runtime()
include(cmake/analyzers.cmake)           # clang-tidy discovery
include(cmake/profiler.cmake)            # Tracy / Perfetto / ClangBuildAnalyzer (when enabled)
include(cmake/configure_target.cmake)    # defines configure_target() — PCH, unity, tidy, profiler link, ASan deploy

#=========================
# Version Header
#=========================
if(GENERATE_VERSION_HEADER)
  generate_version_header()
endif()

#=========================
# Dependencies
#=========================
find_package(fmt CONFIG REQUIRED)
find_package(SDL3 CONFIG REQUIRED)
find_package(CLI11 CONFIG REQUIRED)


#=========================
# Targets
#=========================
# add_subdirectory(src/fc_sitl_cpp)
# add_subdirectory(src/samples/gamepad_btn_test)
add_subdirectory(src/samples/radio_test)

#=========================
# Tests & Coverage
#=========================
# enable_testing() # ctest / gtest_discover_tests register the unit tests
# add_subdirectory(test)
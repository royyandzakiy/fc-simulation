# cmake/compiler.cmake
# Compiler warning policy + platform/build-type machinery. The C++ standard itself stays in the
# top-level CMakeLists (it's a per-project knob); this is the "set once, rarely touched" part.
# Include AFTER project() (it relies on the compiler being known).

if(WIN32)
  set(CMAKE_DEBUG_POSTFIX d) # e.g. app.exe -> appd.exe in Debug, so configs can coexist
endif()

#-------- COMPILER BADGE GENERATION --------
# Record the toolchain CMake actually configured, so CI can publish accurate, drift-proof
# compiler badges (read by scripts/make-badge.py). Works for every compiler incl. MSVC.
file(WRITE "${CMAKE_BINARY_DIR}/toolchain.txt"
     "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}\n")

#-------- COMPILER FLAGS --------
# Default to RelWithDebInfo for single-config generators when no build type is set, so a bare
# `cmake -S . -B build` isn't an unoptimized, debug-info-less build. (Presets always set one;
# multi-config generators like Visual Studio choose the config at build time.)
get_property(_is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
# Top-level only: a parent project (add_subdirectory) owns the global build type.
if(PROJECT_IS_TOP_LEVEL AND NOT _is_multi_config AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      RelWithDebInfo
      CACHE STRING "Build type" FORCE)
  message(STATUS "No CMAKE_BUILD_TYPE set; defaulting to RelWithDebInfo")
endif()

if(MSVC)
  # Baseline: warning level + conformance + UTF-8 (always on).
  add_compile_options(/W4 /permissive- /utf-8)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # cl.exe-only conformance (correctness, esp. /Zc:__cplusplus for feature detection).
    # clang-cl already conforms and would report these as unused args under /WX.
    add_compile_options(/Zc:preprocessor /Zc:__cplusplus)
  endif()
  if(ENABLE_STRICT_COMPILER)
    add_compile_options(/WX) # warnings as errors
  endif()
else()
  # Baseline: warnings visible (+ two cheap, high-value, low-noise ones).
  add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor)
  if(ENABLE_STRICT_COMPILER)
    add_compile_options(-Werror -pedantic) # errors + ISO-conformance pedantry
  endif()
endif()

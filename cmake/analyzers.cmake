# cmake/analyzers.cmake
# Static analysis tools: clang-tidy discovery.
#
# Finds clang-tidy and caches the path as CLANG_TIDY_EXE.
# The actual per-target application happens in configure_target() (cmake/configure_target.cmake),
# so only first-party production code is linted — never tests or third-party FetchContent deps.
#
# Set ENABLE_CLANG_TIDY=ON to activate.

if(ENABLE_CLANG_TIDY)
  find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
  if(CLANG_TIDY_EXE)
    # NB: applied PER-TARGET in configure_target(), not globally — so tidy lints first-party
    # production code only, never the unit tests or FetchContent'd deps (gtest/gmock/sml).
    # A global CMAKE_CXX_CLANG_TIDY tidies third-party sources too, which fails under
    # WarningsAsErrors. CLANG_TIDY_EXE is cached, so configure_target() can read it.
    message(STATUS "Clang-Tidy found: ${CLANG_TIDY_EXE}")
  else()
    message(WARNING "Clang-Tidy not found. Static analysis is disabled.")
  endif()
endif()

# cmake/coverage.cmake
# Code-coverage instrumentation plus a `coverage` target that runs the tests and reports.
#
#   cmake --preset clang-linux-debug -D ENABLE_COVERAGE=ON   # the *-debug presets already set it
#   cmake --build build/<preset> --target coverage
#
# Clang/AppleClang use LLVM source-based coverage (precise line/region/branch data) via
# llvm-profdata + llvm-cov. GCC uses gcov-style instrumentation summarised by gcovr.
# The target builds the test target, runs ctest under instrumentation, prints a per-file
# summary, and writes an HTML report to build/<preset>/coverage-html/.
#
# setup_coverage_target(<test_target>) is called from CMakeLists.txt AFTER the test target
# exists (it needs $<TARGET_FILE:...>). Instrumentation flags below are applied at include
# time so every target built afterwards is instrumented.

if(NOT ENABLE_COVERAGE)
  return()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang") # Clang or AppleClang
  add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
  add_link_options(-fprofile-instr-generate -fcoverage-mapping)
  # Match the LLVM tools to the compiler version (e.g. llvm-cov-21 for Clang 21).
  string(REGEX MATCH "^[0-9]+" _clang_major "${CMAKE_CXX_COMPILER_VERSION}")
  find_program(LLVM_PROFDATA NAMES llvm-profdata-${_clang_major} llvm-profdata)
  find_program(LLVM_COV NAMES llvm-cov-${_clang_major} llvm-cov)
  if(LLVM_PROFDATA AND LLVM_COV)
    message(STATUS "Coverage: LLVM source-based (${LLVM_COV})")
  else()
    message(WARNING "Coverage: llvm-profdata/llvm-cov not found; 'coverage' target unavailable.")
  endif()

elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(--coverage)
  add_link_options(--coverage)
  find_program(GCOVR gcovr)
  if(GCOVR)
    message(STATUS "Coverage: gcov via gcovr (${GCOVR})")
  else()
    message(WARNING "Coverage: gcovr not found (pip install gcovr); 'coverage' target unavailable.")
  endif()

else()
  message(WARNING "Coverage: unsupported compiler '${CMAKE_CXX_COMPILER_ID}'; ignoring ENABLE_COVERAGE.")
endif()

function(setup_coverage_target test_target)
  set(_cov "${CMAKE_BINARY_DIR}/coverage")
  set(_html "${CMAKE_BINARY_DIR}/coverage-html")
  set(_ignore "(/_deps/|/test/|/usr/)") # report first-party production sources only

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND LLVM_PROFDATA AND LLVM_COV)
    add_custom_target(
      coverage
      USES_TERMINAL
      COMMENT "Coverage: running tests + llvm-cov report"
      COMMAND "${CMAKE_COMMAND}" -E rm -rf "${_cov}"
      COMMAND "${CMAKE_COMMAND}" -E make_directory "${_cov}"
      # Each test process writes its own .profraw (%p = pid).
      COMMAND "${CMAKE_COMMAND}" -E env "LLVM_PROFILE_FILE=${_cov}/%p.profraw"
              "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}" --output-on-failure
      # Merge raw profiles (glob needs a shell — coverage is a Clang/Unix path).
      COMMAND bash -c "'${LLVM_PROFDATA}' merge -sparse ${_cov}/*.profraw -o '${_cov}/merged.profdata'"
      # Emit an lcov tracefile (consumed by Codecov/Coveralls and scripts/cov-to-md.py).
      COMMAND bash -c "'${LLVM_COV}' export -format=lcov '$<TARGET_FILE:${test_target}>' -instr-profile='${_cov}/merged.profdata' -ignore-filename-regex='${_ignore}' > '${CMAKE_BINARY_DIR}/coverage.lcov'"
      COMMAND "${LLVM_COV}" report "$<TARGET_FILE:${test_target}>"
              "-instr-profile=${_cov}/merged.profdata" "-ignore-filename-regex=${_ignore}"
      COMMAND "${LLVM_COV}" show "$<TARGET_FILE:${test_target}>"
              "-instr-profile=${_cov}/merged.profdata" "-ignore-filename-regex=${_ignore}"
              -format=html "-output-dir=${_html}"
      COMMAND "${CMAKE_COMMAND}" -E echo "HTML report: ${_html}/index.html"
      DEPENDS ${test_target}
      VERBATIM)

  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND GCOVR)
    add_custom_target(
      coverage
      USES_TERMINAL
      COMMENT "Coverage: running tests + gcovr report"
      COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}" --output-on-failure
      COMMAND "${CMAKE_COMMAND}" -E make_directory "${_html}"
      COMMAND "${GCOVR}" --root "${CMAKE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}" --exclude ".*/_deps/.*"
              --exclude ".*/test/.*" --print-summary --html-details "${_html}/index.html"
      COMMAND "${CMAKE_COMMAND}" -E echo "HTML report: ${_html}/index.html"
      DEPENDS ${test_target}
      VERBATIM)
  endif()
endfunction()

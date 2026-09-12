# cmake/sanitizers.cmake
# Sanitizer configuration: ASan, TSan, MSan, UBSan, and Windows-specific deployment.
#
# Sets compiler/linker flags globally when ENABLE_SANITIZERS is ON.
# Provides deploy_asan_runtime(<target>) for Windows ASan DLL deployment.

# ====== RUNTIME SANITIZERS ======
if(ENABLE_SANITIZERS)
  message(STATUS "Configuring Sanitizer Baseline")
  if(MSVC)
    # Windows (MSVC & clang-cl): ASan is the only real sanitizer — no TSan/MSan/LSan.
    if(ENABLE_ASAN)
      message(STATUS "Sanitizers: ASan (Windows)")
      # /RTC1 (CMake's default Debug flag) is incompatible with ASan on cl and clang-cl — strip it.
      string(REGEX REPLACE "/RTC[1csu]+" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
      string(REGEX REPLACE "/RTC[1csu]+" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
      add_compile_options(/fsanitize=address) # instrumenting; embeds the ASan lib directives
      add_link_options(/INCREMENTAL:NO)        # ASan is incompatible with incremental linking
      # Prebuilt deps (fmt, etc. from vcpkg/conan) aren't ASan-instrumented, so the MSVC STL's
      # container annotations mismatch at link (LNK2038 annotate_string/annotate_vector). Disable
      # them so instrumented and non-instrumented code agree. (Loses container-overflow checks;
      # heap/use-after-free detection is unaffected.)
      add_compile_definitions(_DISABLE_STRING_ANNOTATION _DISABLE_VECTOR_ANNOTATION)
      # NOTE: do NOT pass /fsanitize=address as a *link* option — CMake links MSVC-style via
      # lld-link/link.exe directly (not the clang-cl driver), which rejects it.
      if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        # clang-cl ASan can't use the debug CRT (/MDd) — use the release DLL CRT (/MD).
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
        message(STATUS "  clang-cl: /MD (release CRT) required by ASan")
        # The auto-linked ASan import libs live in clang's runtime dir, which the direct linker
        # invocation doesn't search — add it. (cl.exe's ASan libs are already on the LIB path.)
        execute_process(
          COMMAND "${CMAKE_CXX_COMPILER}" --print-runtime-dir
          OUTPUT_VARIABLE _asan_rt OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(_asan_rt)
          get_filename_component(_asan_rt_parent "${_asan_rt}" DIRECTORY)
          set(_asan_win "${_asan_rt_parent}/windows")
          # Explicitly link the dynamic ASan import lib + runtime thunk (the clang-cl driver
          # would do this; the direct linker invocation won't).
          add_link_options(
            "/LIBPATH:${_asan_win}"
            clang_rt.asan_dynamic-x86_64.lib
            -wholearchive:clang_rt.asan_dynamic_runtime_thunk-x86_64.lib)
        endif()
      endif()
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
      # No ASan requested -> fall back to MSVC runtime checks. /RTC1 is cheap, but is
      # INCOMPATIBLE with /fsanitize=address, so it must never be combined with ASan.
      message(STATUS "Sanitizers: MSVC /RTC1 runtime checks (set ENABLE_ASAN for AddressSanitizer)")
      add_compile_options(/RTC1)
    endif()
    if(ENABLE_TSAN OR ENABLE_MSAN)
      message(WARNING "TSan/MSan are unavailable on Windows (MSVC/clang-cl) — ignoring.")
    endif()
  else()
    # Baseline: UBSan (composes with one of ASan/TSan/MSan).
    # -fno-sanitize-recover=all makes UBSan findings fatal (non-zero exit) instead of just
    # printing and continuing — important so they actually fail a build/test.
    set(SANITIZER_FLAGS
        -fsanitize=undefined
        -fsanitize=bounds
        -fno-sanitize-recover=all
        -fno-omit-frame-pointer
        -fno-optimize-sibling-calls
        -fstack-protector-strong)

    # ASan / TSan / MSan are mutually exclusive — allow at most one.
    set(_san_choice 0)
    foreach(_s ENABLE_ASAN ENABLE_TSAN ENABLE_MSAN)
      if(${_s})
        math(EXPR _san_choice "${_san_choice} + 1")
      endif()
    endforeach()
    if(_san_choice GREATER 1)
      message(FATAL_ERROR "ENABLE_ASAN / ENABLE_TSAN / ENABLE_MSAN are mutually exclusive — enable only one.")
    endif()

    if(ENABLE_ASAN)
      message(STATUS "Sanitizers: UBSan + ASan + LSan")
      list(APPEND SANITIZER_FLAGS -fsanitize=address -fsanitize=leak)
    elseif(ENABLE_TSAN)
      message(STATUS "Sanitizers: UBSan + TSan")
      list(APPEND SANITIZER_FLAGS -fsanitize=thread)
    elseif(ENABLE_MSAN)
      if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "ENABLE_MSAN requires Clang — MemorySanitizer is Clang-only (compiler: ${CMAKE_CXX_COMPILER_ID}).")
      endif()
      message(STATUS "Sanitizers: UBSan + MSan (needs an instrumented libc++ to avoid false positives)")
      list(APPEND SANITIZER_FLAGS -fsanitize=memory -fsanitize-memory-track-origins)
    else()
      message(STATUS "Sanitizers: UBSan baseline only (set ENABLE_ASAN / ENABLE_TSAN / ENABLE_MSAN for more)")
    endif()

    add_compile_options(${SANITIZER_FLAGS})
    add_link_options(${SANITIZER_FLAGS})

    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
      message(WARNING "Sanitizers work best in a Debug build for readable symbols.")
    endif()
  endif()
endif()

# ----- Deploy the ASan runtime DLL (Windows only) -----
# On Windows, ASan links a dynamic runtime DLL (clang_rt.asan_dynamic-x86_64.dll) that must sit
# beside every .exe. Multiple executables in the same output directory cause a file-locking race
# during parallel builds. To avoid this, we deploy the DLL exactly once via a custom target in
# the root CMakeLists scope. All ASan executables then add a dependency on this target.
#
# Usage:
#   deploy_asan_runtime(<target>)   — adds a dependency from <target> to the deploy step
#   Call this AFTER the target is created (e.g. from configure_target()).

if(NOT DEFINED __SANITIZERS_CMAKE_INCLUDED__)
  set(__SANITIZERS_CMAKE_INCLUDED__ TRUE)

  # Locate the ASan DLL once at include time.
  if(MSVC AND ENABLE_ASAN)
    set(_asan_dll "clang_rt.asan_dynamic-x86_64.dll")

    # Build the search path.
    get_filename_component(_cxx_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
    set(_search_paths "${_cxx_dir}")
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      execute_process(
        COMMAND "${CMAKE_CXX_COMPILER}" --print-runtime-dir
        OUTPUT_VARIABLE _rt
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
      if(_rt)
        list(APPEND _search_paths "${_rt}")
        get_filename_component(_rt_parent "${_rt}" DIRECTORY)
        list(APPEND _search_paths "${_rt_parent}/windows")
      endif()
    endif()

    find_file(
      ASAN_RUNTIME_DLL
      NAMES ${_asan_dll}
      PATHS ${_search_paths}
      NO_DEFAULT_PATH)

    if(ASAN_RUNTIME_DLL)
      # Determine the shared output directory.
      if(CMAKE_RUNTIME_OUTPUT_DIRECTORY)
        set(_deploy_dir "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
      else()
        set(_deploy_dir "${CMAKE_BINARY_DIR}")
      endif()

      # Create the target in the ROOT source directory to avoid CMP0002 collisions.
      add_custom_target(DeployAsanRuntime ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${ASAN_RUNTIME_DLL}"
          "${_deploy_dir}/${_asan_dll}"
        COMMENT "Deploying ASan runtime DLL (${_asan_dll}) to ${_deploy_dir}"
        BYPRODUCTS "${_deploy_dir}/${_asan_dll}"
      )
    else()
      message(WARNING "ASan runtime DLL (${_asan_dll}) not found; ASan targets may fail to start.")
    endif()
  endif()
endif()

function(deploy_asan_runtime target)
  if(NOT (MSVC AND ENABLE_ASAN))
    return()
  endif()

  if(TARGET DeployAsanRuntime)
    add_dependencies(${target} DeployAsanRuntime)
  endif()
endfunction()
# NOTE: call deploy_asan_runtime(<target>) AFTER the target is created. This module is included
# before executables exist; the app calls it via configure_target() in cmake/configure_target.cmake.
# cmake/config_vcpkg.cmake
# =============================================================================
# VCPKG Configuration
# =============================================================================
if(NOT PKG_MANAGER STREQUAL "vcpkg")
  message(STATUS "Skip vcpkg (PKG_MANAGER=${PKG_MANAGER})")
  return()
endif()

# Set triplet
if(NOT VCPKG_TARGET_TRIPLET)
  if(WIN32)
    set(VCPKG_TARGET_TRIPLET
        "x64-windows"
        CACHE STRING "vcpkg triplet")
  else()
    set(VCPKG_TARGET_TRIPLET
        "x64-linux"
        CACHE STRING "vcpkg triplet")
  endif()
endif()

# Find vcpkg root
# Priority: CMake var (preset/-D) > env var > vcpkg in PATH > common default paths > project subdir > error
if(VCPKG_ROOT_PATH)
  message(STATUS "VCPKG_ROOT_PATH already set: ${VCPKG_ROOT_PATH}")
else()
  message(STATUS "VCPKG_ROOT_PATH not yet set, finding vcpkg root...")

  # A path is a valid vcpkg root only if the toolchain file is present.
  set(_vcpkg_toolchain_rel "scripts/buildsystems/vcpkg.cmake")

  if(DEFINED ENV{VCPKG_ROOT} AND EXISTS "$ENV{VCPKG_ROOT}/${_vcpkg_toolchain_rel}")
    set(VCPKG_ROOT_PATH "$ENV{VCPKG_ROOT}")
    message(STATUS "ENV\{VCPKG_ROOT\}: $ENV{VCPKG_ROOT}")
  elseif(DEFINED ENV{VCPKG_ROOT_PATH} AND EXISTS "$ENV{VCPKG_ROOT_PATH}/${_vcpkg_toolchain_rel}")
    set(VCPKG_ROOT_PATH "$ENV{VCPKG_ROOT_PATH}")
    message(STATUS "ENV\{VCPKG_ROOT_PATH\}: $ENV{VCPKG_ROOT_PATH}")
  else()
    # Warn loudly if an env var is set but stale, so the cause isn't silent.
    if(DEFINED ENV{VCPKG_ROOT})
      message(WARNING "[vcpkg] ENV{VCPKG_ROOT}='$ENV{VCPKG_ROOT}' has no ${_vcpkg_toolchain_rel} — ignoring and auto-detecting.")
    endif()
    if(DEFINED ENV{VCPKG_ROOT_PATH})
      message(WARNING "[vcpkg] ENV{VCPKG_ROOT_PATH}='$ENV{VCPKG_ROOT_PATH}' has no ${_vcpkg_toolchain_rel} — ignoring and auto-detecting.")
    endif()
    # Auto-detect: derive root from vcpkg executable if it's in PATH
    find_program(_vcpkg_exe vcpkg)

    if(_vcpkg_exe)
      get_filename_component(VCPKG_ROOT_PATH "${_vcpkg_exe}" DIRECTORY)
      message(STATUS "vcpkg.exe auto-detected via executable path: ${VCPKG_ROOT_PATH}")
    else()
      # Auto-detect: check common default installation paths
      set(_vcpkg_default_paths
          "C:/vcpkg"
          "C:/tools/vcpkg"
          "$ENV{LOCALAPPDATA}/vcpkg"
          "$ENV{USERPROFILE}/vcpkg"
          "/opt/vcpkg"
          "$ENV{HOME}/vcpkg"
          "/usr/local/vcpkg")
      foreach(_path IN LISTS _vcpkg_default_paths)
        if(EXISTS "${_path}/scripts/buildsystems/vcpkg.cmake")
          set(VCPKG_ROOT_PATH "${_path}")
          message(STATUS "vcpkg auto-detected at default path: ${VCPKG_ROOT_PATH}")
          break()
        endif()
      endforeach()
    endif()
    unset(_vcpkg_exe CACHE)
  endif()

  # Last resort: project-local submodule/clone
  if(NOT VCPKG_ROOT_PATH AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg")
    set(VCPKG_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg")
  endif()

  if(NOT VCPKG_ROOT_PATH)
    message(
      FATAL_ERROR
        "\n"
        "======================================================================\n"
        " ERROR: vcpkg toolchain not found\n"
        "======================================================================\n"
        " Please provide the path to vcpkg using one of the following options:\n\n"
        " 1. Add vcpkg to your system PATH (auto-detected on next configure)\n\n"
        " 2. Configure via CMakeUserPresets.json:\n"
        "    Copy 'CMakeUserPresets.json.example' to 'CMakeUserPresets.json'\n"
        "    and set 'VCPKG_ROOT_PATH' to your vcpkg location:\n"
        "      Linux  :  \"VCPKG_ROOT_PATH\": \"/opt/vcpkg\"\n"
        "      Windows:  \"VCPKG_ROOT_PATH\": \"C:/vcpkg\"\n\n"
        " 3. Pass the path directly to the CMake command:\n"
        "    cmake -DVCPKG_ROOT_PATH=/path/to/vcpkg .\n\n"
        " 4. Create a local configuration file:\n"
        "    Create 'project_options.local.cmake' and add:\n"
        "    set(VCPKG_ROOT_PATH \"/path/to/vcpkg\")\n"
        "----------------------------------------------------------------------\n")
  endif()
endif()

# Set toolchain file
if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE
      "${VCPKG_ROOT_PATH}/scripts/buildsystems/vcpkg.cmake"
      CACHE FILEPATH "vcpkg toolchain file")
endif()

# Status indicators
set(STATUS_OK "✓")
set(STATUS_FAIL "✗")
set(STATUS_WARN "⚠")
set(STATUS_INFO "ℹ")

message(STATUS "")
message(STATUS "╔════════════════════════════════════════════════════════════════════════════╗")
message(STATUS "║                           VCPKG CONFIGURATION                              ║")
message(STATUS "╚════════════════════════════════════════════════════════════════════════════╝")
message(STATUS "")

# Initialize counters
set(VCPKG_CHECKS_PASSED 0)
set(VCPKG_CHECKS_TOTAL 3)

# -----------------------------------------------------------------------------
# Check 1: CMAKE_TOOLCHAIN_FILE
# -----------------------------------------------------------------------------
message(STATUS "┌─ [1/${VCPKG_CHECKS_TOTAL}] CMAKE_TOOLCHAIN_FILE")
if(DEFINED CMAKE_TOOLCHAIN_FILE)
  if(EXISTS "${CMAKE_TOOLCHAIN_FILE}")
    message(STATUS "│   ${STATUS_OK} ${CMAKE_TOOLCHAIN_FILE}")
    math(EXPR VCPKG_CHECKS_PASSED "${VCPKG_CHECKS_PASSED} + 1")
  else()
    message(STATUS "│   ${STATUS_FAIL} ${CMAKE_TOOLCHAIN_FILE} (file not found)")
    message(FATAL_ERROR "│\n╚════════════════════════════════════════════════════════════════════════════╝\n"
                        "CMAKE_TOOLCHAIN_FILE not found! Please verify vcpkg installation path.\n")
  endif()
else()
  message(STATUS "│   ${STATUS_FAIL} CMAKE_TOOLCHAIN_FILE is not set!")
  message(FATAL_ERROR "│\n╚════════════════════════════════════════════════════════════════════════════╝\n"
                      "CMAKE_TOOLCHAIN_FILE must be set! Add to your CMake preset.\n")
endif()
message(STATUS "└─────────────────────────────────────────────────────────────────────────")

# -----------------------------------------------------------------------------
# Check 2: VCPKG_TARGET_TRIPLET
# -----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "┌─ [2/${VCPKG_CHECKS_TOTAL}] VCPKG_TARGET_TRIPLET")
if(DEFINED VCPKG_TARGET_TRIPLET)
  message(STATUS "│   ${STATUS_OK} ${VCPKG_TARGET_TRIPLET}")
  math(EXPR VCPKG_CHECKS_PASSED "${VCPKG_CHECKS_PASSED} + 1")
else()
  message(STATUS "│   ${STATUS_INFO} Using default: ${VCPKG_TARGET_TRIPLET}")
  set(VCPKG_TARGET_TRIPLET
      "${VCPKG_TARGET_TRIPLET}"
      CACHE STRING "vcpkg triplet" FORCE)
  math(EXPR VCPKG_CHECKS_PASSED "${VCPKG_CHECKS_PASSED} + 1")
endif()
message(STATUS "└─────────────────────────────────────────────────────────────────────────")

# -----------------------------------------------------------------------------
# Check 3: vcpkg installation
# -----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "┌─ [3/${VCPKG_CHECKS_TOTAL}] vcpkg Installation")

# Detect Mode
if(VCPKG_MANIFEST_MODE)
  message(STATUS "│   ${STATUS_INFO} Mode: MANIFEST (using vcpkg.json)")
  message(STATUS "│   ${STATUS_OK} dependencies will be managed automatically")
  math(EXPR VCPKG_CHECKS_PASSED "${VCPKG_CHECKS_PASSED} + 1")
else()
  message(STATUS "│   ${STATUS_INFO} Mode: CLASSIC (global installation)")

  set(TRIPLET_DIR "${VCPKG_ROOT_PATH}/installed/${VCPKG_TARGET_TRIPLET}")
  if(EXISTS "${TRIPLET_DIR}")
    message(STATUS "│   ${STATUS_OK} triplet directory: ${TRIPLET_DIR}")
    math(EXPR VCPKG_CHECKS_PASSED "${VCPKG_CHECKS_PASSED} + 1")

    # Quick global package checks if using vcpkg classic mode. If using manifest mode, ignore
    if(EXISTS "${TRIPLET_DIR}/include/fmt")
      message(STATUS "│   ${STATUS_OK} fmt: found")
    else()
      message(STATUS "│   ${STATUS_WARN} fmt: not found (run: ./vcpkg install fmt --triplet ${VCPKG_TARGET_TRIPLET})")
    endif()
    if(EXISTS "${TRIPLET_DIR}/include/gtest")
      message(STATUS "│   ${STATUS_OK} gtest: found")
    else()
      message(
        STATUS "│   ${STATUS_WARN} gtest: not found (run: ./vcpkg install gtest --triplet ${VCPKG_TARGET_TRIPLET})")
    endif()

    # Add more as needed, following this format:
    # if(EXISTS "${TRIPLET_DIR}/include/drogon")
    #     message(STATUS "│   ${STATUS_OK} drogon: found")
    # else()
    #     message(STATUS "│   ${STATUS_WARN} drogon: not found (run: ./vcpkg install drogon --triplet ${VCPKG_TARGET_TRIPLET})")
    # endif()

  else()
    message(STATUS "│   ${STATUS_WARN} triplet directory not found: ${TRIPLET_DIR}")
    message(STATUS "│   ${STATUS_INFO} Run: ./vcpkg install --triplet ${VCPKG_TARGET_TRIPLET}")
    math(EXPR VCPKG_CHECKS_PASSED "${VCPKG_CHECKS_PASSED} + 0")
  endif()
endif()
message(STATUS "└─────────────────────────────────────────────────────────────────────────")

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "╔════════════════════════════════════════════════════════════════════════════╗")
message(STATUS "║                         VCPKG CONFIGURATION SUMMARY                        ║")
message(STATUS "╠════════════════════════════════════════════════════════════════════════════╣")
message(STATUS "║  ${STATUS_OK} Toolchain file       : ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "║  ${STATUS_OK} vcpkg root           : ${VCPKG_ROOT_PATH}")
message(STATUS "║  ${STATUS_OK} Triplet              : ${VCPKG_TARGET_TRIPLET}")
message(STATUS "╠════════════════════════════════════════════════════════════════════════════╣")
message(
  STATUS
    "║  Checks passed: ${VCPKG_CHECKS_PASSED}/${VCPKG_CHECKS_TOTAL} (${VCPKG_CHECKS_TOTAL} critical)				                            ║"
)
if(VCPKG_CHECKS_PASSED EQUAL VCPKG_CHECKS_TOTAL)
  message(STATUS "║  Status: ${STATUS_OK} READY for configuration                                        ║")
else()
  message(STATUS "║  Status: ${STATUS_WARN} Some checks failed - proceed with caution                      ║")
endif()
message(STATUS "╚════════════════════════════════════════════════════════════════════════════╝")
message(STATUS "")

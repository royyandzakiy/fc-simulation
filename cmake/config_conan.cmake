# cmake/config_conan.cmake
# Conan 2 integration via the cmake-conan dependency provider. Included by
# setup_package_manager() when PKG_MANAGER=conan. Works with ANY preset.
#
# Graceful fallback: if Conan's prerequisites (the cmake-conan submodule + a `conan`
# executable) are missing, this falls back to vcpkg for THIS configure instead of hard
# failing. PKG_MANAGER stays 'conan' in the cache, so fixing the environment and
# re-configuring picks Conan back up automatically.

include(FetchContent)
FetchContent_Declare(
  cmake-conan
  GIT_REPOSITORY https://github.com/conan-io/cmake-conan.git
  GIT_TAG        develop2
)
FetchContent_MakeAvailable(cmake-conan)

set(CONAN_PROVIDER "${cmake-conan_SOURCE_DIR}/conan_provider.cmake")

find_program(CONAN_EXECUTABLE NAMES conan)

set(_conan_ready TRUE)
if(NOT EXISTS "${CONAN_PROVIDER}")
  message(WARNING "[conan] cmake-conan submodule not found at ${CONAN_PROVIDER}\n"
                  "        Run: git submodule update --init --recursive")
  set(_conan_ready FALSE)
endif()
if(NOT CONAN_EXECUTABLE)
  message(WARNING "[conan] 'conan' not found on PATH (install Conan 2, e.g. 'pip install conan').")
  set(_conan_ready FALSE)
endif()

if(NOT _conan_ready)
  message(WARNING "[conan] Prerequisites missing — falling back to vcpkg for this configure "
                  "(PKG_MANAGER stays 'conan' in the cache; fix the env and re-configure to use Conan).")
  set(PKG_MANAGER "vcpkg") # local override for this configure only
  # project_options.cmake only defines VCPKG_MANIFEST_MODE when PKG_MANAGER==vcpkg, which it
  # wasn't here — default it so config_vcpkg uses manifest mode (vcpkg.json), not classic.
  if(NOT DEFINED VCPKG_MANIFEST_MODE)
    set(VCPKG_MANIFEST_MODE ON)
  endif()
  include(cmake/config_vcpkg.cmake)
  return()
endif()

# --- Conan is available: register the provider --------------------------------------------
# Consumed at the next project() call. Honor an explicit override if one was supplied.
if(NOT DEFINED CMAKE_PROJECT_TOP_LEVEL_INCLUDES)
  set(CMAKE_PROJECT_TOP_LEVEL_INCLUDES "${CONAN_PROVIDER}")
endif()

# Host profile selection:
#   - clang-cl needs an explicit profile (maps clang-cl -> compiler=msvc + executables); auto-pick
#     cmake/conan-profiles/clang-cl-windows-<buildtype>.ini based on the active preset.
#   - every other compiler: leave unset so cmake-conan auto-detects from the CMake configuration.
# An explicit -DCONAN_HOST_PROFILE=... always wins.
if(NOT CONAN_HOST_PROFILE AND CMAKE_CXX_COMPILER MATCHES "clang-cl")
  string(TOLOWER "${CMAKE_BUILD_TYPE}" _conan_bt)
  set(_conan_prof "${CMAKE_SOURCE_DIR}/cmake/conan-profiles/clang-cl-windows-${_conan_bt}.ini")
  if(EXISTS "${_conan_prof}")
    set(CONAN_HOST_PROFILE "${_conan_prof}" CACHE FILEPATH "Conan host profile (auto-selected for clang-cl)")
  else()
    message(WARNING "[conan] clang-cl detected but no profile at ${_conan_prof}; "
                    "the provider will auto-detect (may mis-map clang-cl to compiler=clang).")
  endif()
endif()

if(CONAN_HOST_PROFILE)
  message(STATUS "[conan] host profile: ${CONAN_HOST_PROFILE}")
else()
  message(STATUS "[conan] no explicit profile — cmake-conan auto-detects from CMake")
endif()
message(STATUS "[conan] provider: ${CONAN_PROVIDER}")

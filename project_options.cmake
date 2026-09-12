# project_options.cmake — shared project defaults (committed, applies to all developers).
#
# These are option() calls: they set a default value that can be overridden by:
#   1. project_options.local.cmake (gitignored, personal) — included BEFORE this file
#   2. CMake presets (CMakePresets.json) — cacheVariables
#   3. Command line: cmake -DENABLE_TRACY=ON ..

# ------ Compiler Warnings ------
option(ENABLE_STRICT_COMPILER "Warnings become errors (-Werror / /WX)" OFF)

# ------ Project Features ------
option(BUILD_APP "Build the application binary (src/myapp)" ON)
option(BUILD_LIB "Build the library (src/mylib)" OFF)
if(BUILD_LIB)
  option(BUILD_LIB_SHARED "Build MyLib as SHARED/DLL (ON) or STATIC (OFF)" ON)
  option(GENERATE_VERSION_HEADER "Generate include/<project>/version.h from version.txt" ON)
endif()
option(BUILD_TESTING "Build the unit tests under test/" OFF)
option(BUILD_EXAMPLES "Build the examples/ demos (MyLib consumer, Tracy/Perfetto examples, …)" OFF)

# ------ Package Manager ------
set(PKG_MANAGER "vcpkg" CACHE STRING "Dependency provider: vcpkg | conan | none")
set_property(CACHE PKG_MANAGER PROPERTY STRINGS vcpkg conan none)
if(PKG_MANAGER STREQUAL "vcpkg")
  option(VCPKG_MANIFEST_MODE "Use vcpkg manifest mode (ON) or classic/global mode (OFF)" ON)
endif()

# ------ Sanitizers ------
# UBSan is the baseline (always enabled with sanitizers); pick AT MOST ONE of ASan / TSan / MSan
# (they are mutually exclusive — you cannot combine them).
option(ENABLE_SANITIZERS "Enable runtime sanitizers (UBSan baseline)" OFF)
if(ENABLE_SANITIZERS)
  option(ENABLE_ASAN "AddressSanitizer + LeakSanitizer — detects buffer overflows, use-after-free, leaks" OFF)
  option(ENABLE_TSAN "ThreadSanitizer — detects data races and deadlocks" OFF)
  option(ENABLE_MSAN "MemorySanitizer — detects uninitialized reads (Clang only, requires instrumented libc++)" OFF)
endif()

# ------ Linters & Static Analyzers ------
option(ENABLE_CLANG_TIDY "Run clang-tidy on first-party production code (no-op if clang-tidy isn't installed)" OFF)

# ------ Coverage ------
option(ENABLE_COVERAGE "Instrument for code coverage and add the 'coverage' target (runs tests + generates report)" OFF)

# ------ Compile-Time Optimizations ------
option(ENABLE_CCACHE "Wrap compiles in ccache — huge speedup when switching branches or re-configuring" OFF)
option(ENABLE_FAST_LINKER "Use mold/lld instead of default linker — dramatically faster linking (no-op if absent)" OFF)
option(ENABLE_PCH "Precompile heavy headers (STL, …) — faster full builds, slower single-file incremental rebuilds" OFF)
option(ENABLE_UNITY_BUILD "Batch .cpp files into unity translation units — fastest clean builds, kills incremental builds" OFF)
option(ENABLE_CLANG_BUILD_ANALYZER "Clang -ftime-trace + ClangBuildAnalyzer target — profile what takes time during compilation (Clang only)" OFF)

# ------ Runtime Profilers ------
option(ENABLE_TRACY "Tracy: real-time low-overhead sampling profiler for runtime (CPU, GPU, memory, locks, zones)" OFF)
option(ENABLE_PERFETTO "Perfetto: runtime tracing & trace analysis (lightweight instrumentation, rich trace viewer)" OFF)

# cmake/build_acceleration.cmake
# Build-speed optimizations: fast linker (mold/lld) and compiler cache (ccache).
#
# These are purely compile-time accelerators — they don't affect runtime behavior,
# instrumentation, or analysis. Safe to enable/disable independently of sanitizers
# or static analyzers.
#
# Set ENABLE_FAST_LINKER=ON and/or ENABLE_CCACHE=ON to activate.

# ----- Fast linker (mold / lld) -----
# Linking is often the dominant cost; swapping the linker is a one-flag win. Applies only to
# GNU-like drivers (Clang/GCC/AppleClang/MinGW); MSVC & clang-cl use their own fast linkers.
if(ENABLE_FAST_LINKER AND NOT MSVC)
  find_program(MOLD_LINKER mold)
  find_program(LLD_LINKER NAMES ld.lld lld)
  if(MOLD_LINKER)
    add_link_options(-fuse-ld=mold)
    message(STATUS "Fast linker: mold (${MOLD_LINKER})")
  elseif(LLD_LINKER)
    add_link_options(-fuse-ld=lld)
    message(STATUS "Fast linker: lld (${LLD_LINKER})")
  else()
    message(STATUS "Fast linker: mold/lld not found; using the default linker")
  endif()
endif()

# ----- ccache -----
if(ENABLE_CCACHE)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    message(STATUS "ccache found: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")

    # ccache can't cache MSVC/clang-cl separate-PDB debug info (/Zi, the Debug default).
    # Embed it (/Z7) on Windows so the clang-cl/msvc toolchains actually get cache hits.
    # Still fully debuggable (cppvsdbg/LLDB). Requires CMake >= 3.25 (CMP0141 NEW).
    if(WIN32)
      set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "Embedded")
      message(STATUS "ccache: using embedded debug info (/Z7) on Windows for cacheability")
    endif()
  else()
    message(STATUS "ccache not found. Rapid recompilation disabled.")
  endif()
endif()

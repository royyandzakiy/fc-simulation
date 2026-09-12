# cmake/configure_target.cmake
# Per-target build configuration for the main executable: precompiled headers, unity build,
# optional profiler links, clang-tidy, and (Windows) ASan runtime deployment.
#
# Call configure_target(<target>) AFTER the target and its link libraries are defined.
#
# Depends on:
#   cmake/analyzers.cmake         — provides CLANG_TIDY_EXE
#   cmake/profiler.cmake          — provides project_tracy_profile / project_perfetto_profile targets
#   cmake/sanitizers.cmake        — provides deploy_asan_runtime()

function(configure_target target)
  # Clang-tidy — applied here so it lints only first-party production targets you opt in by
  # calling configure_target(). Tests and third-party FetchContent code are never tidied
  # (test code legitimately breaks many rules; deps are not ours to lint).
  if(ENABLE_CLANG_TIDY AND CLANG_TIDY_EXE)
    set_target_properties(${target} PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
    message(STATUS "Clang-Tidy enabled for ${target}")
  endif()

  # Precompiled headers — TUNE this list to the heavy headers you include everywhere.
  if(ENABLE_PCH)
    target_precompile_headers(
      ${target}
      PRIVATE
      <filesystem>
      <string>
      <string_view>
      <vector>
      <memory>
      <utility>)
    message(STATUS "PCH enabled for ${target}")
  endif()

  # Unity / jumbo build — only meaningful once the target has several .cpp files.
  if(ENABLE_UNITY_BUILD)
    set_target_properties(${target} PROPERTIES UNITY_BUILD ON)
    message(STATUS "Unity build enabled for ${target}")
  endif()

  # Optional profiler interface targets (created by cmake/profiler.cmake when enabled).
  if(TARGET project_tracy_profile)
    target_link_libraries(${target} PRIVATE project_tracy_profile)
  endif()
  if(TARGET project_perfetto_profile)
    target_link_libraries(${target} PRIVATE project_perfetto_profile)
  endif()

  # Windows ASan: deploy the runtime DLL next to the executable (no-op on Linux/macOS).
  if(ENABLE_ASAN)
    deploy_asan_runtime(${target})
  endif()
endfunction()

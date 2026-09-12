# cmake/clangd.cmake
# Presets build into build/<preset>/, which clangd can't auto-discover (it only searches the
# file's dir, ./build, and ancestors). Mirror the active preset's compile_commands.json to the
# project root so the checked-in .clangd (CompilationDatabase: .) always reflects the last build.
# The Visual Studio and Xcode generators don't emit compile_commands.json — only Ninja/Makefiles —
# so only mirror for generators that produce it.
if(CMAKE_EXPORT_COMPILE_COMMANDS AND NOT CMAKE_GENERATOR MATCHES "Visual Studio|Xcode")
  add_custom_target(
    mirror_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_BINARY_DIR}/compile_commands.json"
            "${CMAKE_SOURCE_DIR}/compile_commands.json"
    BYPRODUCTS "${CMAKE_SOURCE_DIR}/compile_commands.json"
    COMMENT "Mirroring compile_commands.json to project root for clangd"
    VERBATIM)
endif()

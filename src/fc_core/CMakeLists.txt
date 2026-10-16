# Header-only flight core, shared by the host build, the tests and fw/.
add_library(fc_core INTERFACE)
add_library(fc::core ALIAS fc_core)
target_include_directories(fc_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(fc_core INTERFACE cxx_std_20)

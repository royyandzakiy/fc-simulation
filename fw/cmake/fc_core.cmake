# Puts the shared flight core (src/fc_core) on the app's include path.
# Include after find_package(Zephyr), which creates the `app` target.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../src/fc_core/include)

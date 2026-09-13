set(MY_PROJECT_NAME gamepad_btn_test)
add_executable(${MY_PROJECT_NAME})

file(GLOB SOURCES *.cpp)
target_sources(${MY_PROJECT_NAME} PRIVATE ${SOURCES})

target_include_directories(${MY_PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${MY_PROJECT_NAME}
    PRIVATE
        CLI11::CLI11
        fmt::fmt
        SDL3::SDL3
)

set_target_properties(${MY_PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE OFF)
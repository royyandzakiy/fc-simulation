set(MY_PROJECT_NAME ${PROJECT_NAME})
add_executable(${MY_PROJECT_NAME})

file(GLOB SOURCES *.cpp)
target_sources(${MY_PROJECT_NAME} PRIVATE ${SOURCES})

target_include_directories(${MY_PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${MY_PROJECT_NAME}
    PRIVATE
        SDL2::SDL2
        CLI11::CLI11
        fmt::fmt
)
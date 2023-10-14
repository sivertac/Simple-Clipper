cmake_minimum_required(VERSION 3.20)

add_library(SimpleClipper 
    ${CMAKE_CURRENT_SOURCE_DIR}/SimpleClipper.cpp
)

target_include_directories(SimpleClipper PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_dependencies(SimpleClipper wykobi)
target_include_directories(SimpleClipper PUBLIC ${wykobi_INCLUDE_DIR})

add_executable(SimpleClipperMain
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)
target_link_libraries(SimpleClipperMain SimpleClipper)


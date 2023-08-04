cmake_minimum_required(VERSION 3.26.4)

project(PROJECT VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS off)
find_package(SDL2 REQUIRED)

add_executable(${PROJECT_NAME}
    main.cpp
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC /opt/homebrew/Cellar/sdl2/2.28.1/include/
    PUBLIC /opt/homebrew/Cellar/glm/0.9.9.8/include
)

target_link_libraries(${PROJECT_NAME} 
    ${SDL2_LIBRARIES}
)
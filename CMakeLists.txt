cmake_minimum_required(VERSION 3.21)
project(PZ3_Physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ---- SFML 3.0.2 скачивается и собирается автоматически (ставить вручную не нужно) ----
include(FetchContent)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)      # статическая сборка: не нужны DLL рядом с exe
set(SFML_BUILD_AUDIO   OFF CACHE BOOL "" FORCE)     # звук и сеть не нужны
set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG        3.0.2
    GIT_SHALLOW    ON)
FetchContent_MakeAvailable(SFML)

# ---- физика, сцены и эксперименты (без SFML) ----
add_library(pz3_core STATIC
    src/Physics.cpp
    src/Scenes.cpp
    src/Experiments.cpp)
target_include_directories(pz3_core PUBLIC src)
if(MSVC)
    target_compile_options(pz3_core PUBLIC /utf-8 /W3)
endif()

# ---- программа с окном ----
add_executable(pz3 src/main.cpp)
target_link_libraries(pz3 PRIVATE pz3_core SFML::Graphics)

# ---- консольные эксперименты без окна (можно запускать отдельно) ----
add_executable(pz3_experiments src/ExperimentsMain.cpp)
target_link_libraries(pz3_experiments PRIVATE pz3_core)

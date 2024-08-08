cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)

# Check the compiler and set appropriate flags
if (MSVC)
    # MSVC specific flags
    target_compile_options(RayTracing PUBLIC /W4 /permissive-)
    # No equivalent for sanitizers in MSVC
else()
    # GCC/Clang specific flags
    target_compile_options(RayTracing PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
    target_link_libraries(RayTracing PUBLIC -fsanitize=undefined)
endif()

target_compile_features(RayTracing PUBLIC cxx_std_17)


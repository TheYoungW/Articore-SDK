cmake_minimum_required(VERSION 3.16)
project(articore_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(articore_runtime_core STATIC src/runtime.cpp)
target_include_directories(articore_runtime_core PUBLIC include src)
target_compile_features(articore_runtime_core PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(articore_runtime_core PRIVATE /W4)
else()
  target_compile_options(articore_runtime_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(articore_runtime_test tests/runtime_test.cpp)
  target_link_libraries(articore_runtime_test PRIVATE articore_runtime_core)
  add_test(NAME articore_runtime_test COMMAND articore_runtime_test)
endif()

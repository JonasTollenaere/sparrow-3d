project(sparrow-3d)

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

### CPP ###
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

### CGAL ###
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE "TRUE")
find_package(CGAL REQUIRED)
link_libraries(CGAL::CGAL)
include_directories(${CGAL_INCLUDE_DIRS})

### MeshCore ###
add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)
set(MESHCORE_BUILD_DEMOS OFF)
include(FetchContent)
set(CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
FetchContent_Declare(MeshCore
        GIT_REPOSITORY git@github.com:JonasTollenaere/MeshCore.git
        GIT_TAG        71cd02892c736843ad8c4c3f7064b5d7adef165f
)
FetchContent_MakeAvailable(MeshCore)

file(GLOB SOURCES src/*.cpp)
include_directories(include)
add_compile_definitions(SOLUTION_DIR="${CMAKE_CURRENT_SOURCE_DIR}/solutions/")

add_executable(StripPacking_Fixed_Rotation StripPacking_Fixed_Rotation.cpp ${SOURCES})
target_link_libraries(StripPacking_Fixed_Rotation PUBLIC MeshCore)

add_executable(StripPacking_Discrete_Rotation StripPacking_Discrete_Rotation.cpp ${SOURCES})
target_link_libraries(StripPacking_Discrete_Rotation PUBLIC MeshCore)

add_executable(Benchmark_Fixed_Rotation Benchmark_Fixed_Rotation.cpp ${SOURCES})
target_link_libraries(Benchmark_Fixed_Rotation PUBLIC MeshCore)

add_executable(Benchmark_Discrete_Rotation Benchmark_Discrete_Rotation.cpp ${SOURCES})
target_link_libraries(Benchmark_Discrete_Rotation PUBLIC MeshCore)

add_executable(RenderPolesOfInaccessibility RenderPolesOfInaccessibility.cpp ${SOURCES})
target_link_libraries(RenderPolesOfInaccessibility PUBLIC MeshCore)
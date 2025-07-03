project(packer)

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

### CPP ###
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

file(GLOB SOURCES AlphaWrapper.cpp EnhancedStripPackingSolution.cpp InaccessibilityPoles.cpp)

### CGAL ###
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE "TRUE")
find_package(CGAL REQUIRED)
link_libraries(CGAL::CGAL)
include_directories(${CGAL_INCLUDE_DIRS})

### MeshCore ###
add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)
set(CMAKE_TOOLCHAIN_FILE="C:/Users/Jonas/.vcpkg-clion/vcpkg-mip/scripts/buildsystems/vcpkg.cmake")
include(FetchContent)
FetchContent_Declare(MeshCore
        GIT_REPOSITORY git@gitlab.kuleuven.be:codes/research/meshcore.git
        GIT_TAG        embed-optimization
)
FetchContent_MakeAvailable(MeshCore)

add_executable(packer main.cpp ${SOURCES})
target_link_libraries(packer PUBLIC MeshCore)

add_executable(StripPacking_GLS StripPacking_GLS.cpp ${SOURCES})
target_link_libraries(StripPacking_GLS PUBLIC MeshCore)

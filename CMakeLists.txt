cmake_minimum_required(VERSION 3.16)
project(PlaneSpotter VERSION 1.0.0 LANGUAGES CXX)

# ──────────────────────────────────────────────
# C++ Standard
# ──────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ──────────────────────────────────────────────
# Output directories
# ──────────────────────────────────────────────
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ──────────────────────────────────────────────
# Find packages
# ──────────────────────────────────────────────

# OpenCV — prefer apt system install over any local CUDA-enabled build
if(NOT DEFINED OpenCV_DIR)
    set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4" CACHE PATH "OpenCV cmake config dir")
endif()
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV found: ${OpenCV_VERSION}")

include(FetchContent)

# Eigen — header-only linear algebra library
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(Eigen3_FOUND)
    message(STATUS "Eigen found via find_package: ${Eigen3_VERSION}")
    set(EIGEN_TARGET Eigen3::Eigen)
else()
    set(LOCAL_EIGEN_PATH "${CMAKE_SOURCE_DIR}/../eigen")
    if(NOT EXISTS "${LOCAL_EIGEN_PATH}/Eigen")
        set(LOCAL_EIGEN_PATH "$ENV{HOME}/repos/eigen")
    endif()

    if(EXISTS "${LOCAL_EIGEN_PATH}/Eigen")
        message(STATUS "Eigen found locally at ${LOCAL_EIGEN_PATH}")
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${LOCAL_EIGEN_PATH}")
        set(EIGEN_TARGET Eigen3::Eigen)
    else()
        FetchContent_Declare(eigen
            URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        )
        FetchContent_MakeAvailable(eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}")
        set(EIGEN_TARGET Eigen3::Eigen)
        message(STATUS "Eigen fetched via FetchContent")
    endif()
endif()

# nlohmann/json
find_package(nlohmann_json QUIET)
if(nlohmann_json_FOUND)
    message(STATUS "nlohmann/json found via find_package")
else()
    FetchContent_Declare(nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    )
    FetchContent_MakeAvailable(nlohmann_json)
    message(STATUS "nlohmann/json fetched via FetchContent")
endif()

# cpr — libcurl wrapper for HTTP requests (AeroAPI / FlightAware)
find_package(cpr QUIET)
if(cpr_FOUND)
    message(STATUS "cpr found via find_package: ${cpr_VERSION}")
else()
    FetchContent_Declare(cpr
        GIT_REPOSITORY https://github.com/libcpr/cpr.git
        GIT_TAG 1.11.2
    )
    FetchContent_MakeAvailable(cpr)
    message(STATUS "cpr fetched via FetchContent")
endif()

# GeographicLib — apt package ships a find-module, not a config file
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
message(STATUS "GeographicLib found: ${GeographicLib_LIBRARIES}")

# GoogleTest
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/releases/download/v1.15.2/googletest-1.15.2.tar.gz
)
FetchContent_MakeAvailable(googletest)

# ──────────────────────────────────────────────
# Shared library (all sources except main.cpp)
# ──────────────────────────────────────────────
file(GLOB_RECURSE LIB_SOURCES
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.cxx"
)
list(FILTER LIB_SOURCES EXCLUDE REGEX ".*/main\\.cpp$")

add_library(PlaneSpotterLib STATIC ${LIB_SOURCES})

target_include_directories(PlaneSpotterLib
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
)

target_link_libraries(PlaneSpotterLib
    PUBLIC
        ${OpenCV_LIBS}
        nlohmann_json::nlohmann_json
        ${GeographicLib_LIBRARIES}
        ${EIGEN_TARGET}
        cpr::cpr
)

target_compile_options(PlaneSpotterLib PRIVATE -Wall -Wextra -Wpedantic)

# ──────────────────────────────────────────────
# Main executable
# ──────────────────────────────────────────────
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE PlaneSpotterLib)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

# ──────────────────────────────────────────────
# Tests
# ──────────────────────────────────────────────
enable_testing()

file(GLOB_RECURSE TEST_SOURCES "${CMAKE_SOURCE_DIR}/tests/*.cpp")
add_executable(PlaneSpotterTests ${TEST_SOURCES})

target_link_libraries(PlaneSpotterTests
    PRIVATE
        PlaneSpotterLib
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(PlaneSpotterTests)

# ──────────────────────────────────────────────
# Build type default
# ──────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

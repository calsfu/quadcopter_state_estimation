cmake_minimum_required(VERSION 3.10)
project(DroneStateEstimation VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(WEBOTS_HOME "/usr/local/webots")

# Find Eigen package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Manually include the Webots directories
include_directories(/usr/local/webots/include)
include_directories(include)  # Add this line to include your project's header files

# Link the Webots controller libraries
link_directories(${WEBOTS_HOME}/lib)

# Specify where the source files are
file(GLOB_RECURSE SOURCES src/*.cpp)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link Eigen and Webots libraries
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${WEBOTS_HOME}/lib/controller/libCppController.so)
target_include_directories(${PROJECT_NAME} PRIVATE ${WEBOTS_HOME}/include/controller/cpp)

# Set output directories for runtime binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

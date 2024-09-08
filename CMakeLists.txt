cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 20)
set(WEBOTS_HOME "/usr/local/webots")
set(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH})
set(Eigen3_DIR "/usr/include/eigen-3.4.0")

project( QUADCONTROL )

find_package(Eigen3  REQUIRED )
find_package(manif  REQUIRED )

file(GLOB QUADCONTROL_SRC
     "include/*.h"
     "src/*.cpp"
)

link_directories(${WEBOTS_HOME}/lib)
add_executable( quad ${QUADCONTROL_SRC} )
target_link_libraries(quad ${WEBOTS_HOME}/lib/controller/libCppController.so pthread)
target_include_directories( quad SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${manif_INCLUDE_DIRS} ${WEBOTS_HOME}/include/controller/cpp include/)
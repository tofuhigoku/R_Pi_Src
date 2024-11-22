cmake_minimum_required(VERSION 3.14.0)
project (test_matplotlib)

find_package(Python3 3.8 EXACT COMPONENTS Intrepreter Development NumPy)

# set(PROJECTS_LINK_LIBS )
# link_directories( /usr/include)

set(CMAKE_CXX_FLAGS "-Wall -std=c++17  -lpthread -lrt")

include_directories(${Python3_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../dogo/src/include)

# file(GLOB SOURCES1 "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

# set(SOURCES)

add_executable(test1 ${SOURCES})
target_link_libraries(test1 ${Python3_LIBRARIES})
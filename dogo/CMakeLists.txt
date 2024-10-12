cmake_minimum_required(VERSION 2.8.0)
project (test1)

# set(PROJECTS_LINK_LIBS glfw GLEW GL GLU)
# link_directories( /usr/lib)

set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -lpthread -lrt")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

file(GLOB SOURCES1 "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

# set(SOURCES)

add_executable(test1 ${SOURCES} ${SOURCES1})
# target_link_libraries(GL_Texture ${PROJECTS_LINK_LIBS})
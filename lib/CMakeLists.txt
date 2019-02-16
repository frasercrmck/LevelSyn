cmake_minimum_required(VERSION 2.6)
set( CMAKE_CXX_STANDARD 17 )
project(LevelSyn)

set(Boost_USE_STATIC_LIBS TRUE)
find_package(Boost COMPONENTS thread REQUIRED)
link_directories    (${Boost_LIBRARY_DIRS})
include_directories (${Boost_INCLUDE_DIRS})

file(GLOB LEVEL_SYN_SOURCES "*.cpp")
file(GLOB LEVEL_SYN_HEADERS "*.h")

add_library(LevelSyn STATIC ${LEVEL_SYN_SOURCES} )

target_include_directories(LevelSyn
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  SYSTEM
    "${Boost_INCLUDE_DIRS}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/tinyxml"
    "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/clipper"
)

target_link_libraries(LevelSyn tinyxml clipper ${Boost_LIBRARIES})

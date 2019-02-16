cmake_minimum_required(VERSION 2.6)
set( CMAKE_CXX_STANDARD 17 )

if(APPLE)
  find_library(coreFoundation CoreFoundation REQUIRED)
endif()

add_executable(levels levels.cpp)

target_link_libraries(levels LevelSyn)

if(APPLE)
   target_link_libraries(levels ${coreFoundation})
endif()

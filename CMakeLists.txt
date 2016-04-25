cmake_minimum_required(VERSION 2.8)

option(WithSerialization "Compile with boost::serialization" OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Compile with Myo library.
find_package(Myo REQUIRED)
include_directories(${Myo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Myo_LIBRARIES})

# Compile with boost serialization library
# if -DWithSerialization=On is specified on the command line.
if(WithSerialization)
  find_package(Boost REQUIRED serialization)
  include_directories(${Boost_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${Boost_LIBRARIES})
endif(WithSerialization)

add_subdirectory(src)
add_subdirectory(examples)

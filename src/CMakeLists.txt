option(WithSerialization "Compile with boost::serialization" OFF)
set(SRCS ${SRCS} Hub.cpp EventPlayerHub.cpp EventRecorder.cpp)

# Compile with Myo library.
find_package(Myo REQUIRED)
include_directories(${Myo_INCLUDE_DIRS})
set(LIBS ${LIBS} ${Myo_LIBRARIES})

# Compile with boost serialization library if -DWithSerialization=YES is specified on the command line.
if(WithSerialization)
  find_package(Boost REQUIRED serialization)
  include_directories(${Boost_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${Boost_LIBRARIESS})
  set(SRCS ${SRCS} SerializableEventTypes.cpp)
endif(WithSerialization)

add_library(myosim ${SRCS})
target_link_libraries(myosim ${LIBS})
target_compile_features(myosim PRIVATE cxx_auto_type) # C++11

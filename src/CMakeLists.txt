set(SRCS ${SRCS} Hub.cpp EventPlayerHub.cpp EventRecorder.cpp)
# Compile with boost serialization library if -DWithSerialization=YES is specified on the command line.
if(WithSerialization)
  set(SRCS ${SRCS} SerializableEventTypes.cpp)
endif(WithSerialization)

add_library(myosim ${SRCS})
target_link_libraries(myosim ${LIBS})
target_compile_features(myosim PRIVATE cxx_auto_type) # C++11

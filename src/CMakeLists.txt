set(SRCS ${SRCS} hub.cpp event_player_hub.cpp event_recorder.cpp)
# Compile with boost serialization library if -DWithSerialization=YES is specified on the command line.
if(WithSerialization)
  set(SRCS ${SRCS} serializable_event_types.cpp)
endif(WithSerialization)

add_library(myosim ${SRCS})
target_link_libraries(myosim ${LIBS})
target_compile_features(myosim PRIVATE cxx_auto_type) # C++11

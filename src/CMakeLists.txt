set(SRCS hub.cpp event_player_hub.cpp event_recorder.cpp)
set(HEAD hub.h 	 event_player_hub.h   event_recorder.h)
# Compile with boost serialization library if -DWithSerialization=On
# is specified on the command line.
if(WithSerialization)
  set(SRCS ${SRCS} serializable_event_types.cpp)
  set(HEAD ${HEAD} serializable_event_types.h)
endif(WithSerialization)

add_library(myosim ${SRCS})
target_link_libraries(myosim ${LIBS})
target_compile_features(myosim PRIVATE cxx_auto_type) # C++11

# interpreted relative to CMAKE_INSTALL_PREFIX
install(FILES ${HEAD} DESTINATION ./include)
install(TARGETS myosim DESTINATION ./lib)

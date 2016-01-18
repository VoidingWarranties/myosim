option(InteractiveExample "Compile interactive example" OFF)
option(PlaybackExample "Compile playback example" OFF)
option(SerializationExample "Compile serialization example" OFF)

set(LIBS ${LIBS} myosim)

if(InteractiveExample)
  add_executable(interactive interactive.cpp)
  target_link_libraries(interactive ${LIBS})
  target_compile_features(interactive PRIVATE cxx_auto_type) # C++11
endif(InteractiveExample)

if(PlaybackExample)
  add_executable(playback playback.cpp)
  target_link_libraries(playback ${LIBS})
  target_compile_features(playback PRIVATE cxx_auto_type) # C++11
endif(PlaybackExample)

if(SerializationExample)
  add_executable(serialization serialize.cpp)
  target_link_libraries(serialization ${LIBS})
  target_compile_features(serialization PRIVATE cxx_auto_type) # C++11
endif(SerializationExample)

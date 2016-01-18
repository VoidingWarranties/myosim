option(InteractiveExample "Compile interactive example" OFF)
option(PlaybackExample "Compile playback example" OFF)
option(SerializationExample "Compile serialization example" OFF)

set(LIBS ${LIBS} myosim)

if(InteractiveExample)
  add_executable(Interactive Interactive.cpp)
  target_link_libraries(Interactive ${LIBS})
  target_compile_features(Interactive PRIVATE cxx_auto_type) # C++11
endif(InteractiveExample)

if(PlaybackExample)
  add_executable(Playback Playback.cpp)
  target_link_libraries(Playback ${LIBS})
  target_compile_features(Playback PRIVATE cxx_auto_type) # C++11
endif(PlaybackExample)

if(SerializationExample)
  add_executable(Serialization Serialize.cpp)
  target_link_libraries(Serialization ${LIBS})
  target_compile_features(Serialization PRIVATE cxx_auto_type) # C++11
endif(SerializationExample)

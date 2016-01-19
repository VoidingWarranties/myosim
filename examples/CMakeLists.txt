set(LIBS ${LIBS} myosim)

add_executable(interactive-example EXCLUDE_FROM_ALL interactive.cpp)
set_target_properties(interactive-example PROPERTIES OUTPUT_NAME interactive)
target_link_libraries(interactive-example ${LIBS})
target_compile_features(interactive-example PRIVATE cxx_auto_type) # C++11

add_executable(playback-example EXCLUDE_FROM_ALL playback.cpp)
set_target_properties(playback-example PROPERTIES OUTPUT_NAME playback)
target_link_libraries(playback-example ${LIBS})
target_compile_features(playback-example PRIVATE cxx_auto_type) # C++11

add_executable(serialization-example EXCLUDE_FROM_ALL serialize.cpp)
set_target_properties(serialization-example PROPERTIES OUTPUT_NAME serialization)
target_link_libraries(serialization-example ${LIBS})
target_compile_features(serialization-example PRIVATE cxx_auto_type) # C++11

find_package(Myo REQUIRED)
include_directories(${Myo_INCLUDE_DIRS})
add_library(myosim Hub.cpp EventPlayerHub.cpp EventRecorder.cpp)
target_compile_features(myosim PRIVATE cxx_auto_type) # C++11

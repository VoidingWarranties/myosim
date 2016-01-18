find_package(PkgConfig)

find_path(Myo_INCLUDE_DIR NAMES myo/myo.hpp)
find_library(Myo_LIBRARY NAMES myo)

set(Myo_INCLUDE_DIRS ${Myo_INCLUDE_DIR})
set(Myo_LIBRARIES ${Myo_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Myo DEFAULT_MSG Myo_INCLUDE_DIR Myo_LIBRARY)

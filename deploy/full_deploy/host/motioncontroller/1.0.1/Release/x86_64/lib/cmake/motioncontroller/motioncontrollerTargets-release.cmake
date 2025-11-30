#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "MC::motioncontroller" for configuration "Release"
set_property(TARGET MC::motioncontroller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(MC::motioncontroller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmotioncontroller.so"
  IMPORTED_SONAME_RELEASE "libmotioncontroller.so"
  )

list(APPEND _cmake_import_check_targets MC::motioncontroller )
list(APPEND _cmake_import_check_files_for_MC::motioncontroller "${_IMPORT_PREFIX}/lib/libmotioncontroller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

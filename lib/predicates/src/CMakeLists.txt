#------------------------------------------------------------------------------
# Macro set set the library properties
macro(LibraryProperties targetName DEBUG_EXTENSION)
  if(MSVC)
    SET_TARGET_PROPERTIES( ${targetName}
      PROPERTIES
      DEBUG_OUTPUT_NAME lib${targetName}
      RELEASE_OUTPUT_NAME lib${targetName}  )
  endif()

  set_target_properties( ${targetName} PROPERTIES FOLDER ${targetName}Proj)


  #-- Set the Debug and Release names for the libraries
  SET_TARGET_PROPERTIES( ${targetName}
      PROPERTIES
      DEBUG_POSTFIX ${DEBUG_EXTENSION} )
endmacro(LibraryProperties DEBUG_EXTENSION)



set (predicates_SRCS
  ${predicates_SOURCE_DIR}/src/constants.c
  ${predicates_SOURCE_DIR}/src/predicates.c
  ${predicates_SOURCE_DIR}/src/random.c
  )

set(predicates_HDRS
  ${predicates_SOURCE_DIR}/src/constants.h
  ${predicates_SOURCE_DIR}/include/predicates.h
  ${predicates_SOURCE_DIR}/include/predicates.hpp
  ${predicates_SOURCE_DIR}/include/predicatesDLLExport.h
)

add_library (predicates ${LIB_TYPE} ${predicates_SRCS} ${predicates_HDRS})
LibraryProperties(predicates "_d")

set(install_dir "bin")
set(lib_install_dir "lib")

install (TARGETS predicates 
          COMPONENT Applications
          EXPORT PredicatesLibTargets
          RUNTIME DESTINATION ${install_dir}
          LIBRARY DESTINATION ${lib_install_dir}
          ARCHIVE DESTINATION ${lib_install_dir}
          BUNDLE DESTINATION ".")


# --------------------------------------------------------------------
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/Predicates/PredicatesLibTargetsConfigVersion.cmake"
  VERSION ${Predicates_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_file(cmake/PredicatesConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/Predicates/PredicatesConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation share/Predicates)

install(
  FILES
    cmake/PredicatesConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/Predicates/PredicatesLibTargetsConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)



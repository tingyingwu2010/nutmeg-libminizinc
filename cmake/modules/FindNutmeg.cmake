# - Try to find Nutmeg
# Once done this will define
#  NUTMEG_FOUND        - System has Nutmeg
#  NUTMEG_INCLUDE_DIRS - The Nutmeg include directories
#  NUTMEG_LIBRARIES    - The libraries needed to use Nutmeg
# User can set NUTMEG_ROOT to the preferred installation prefix
# Imported target Nutmeg will be created for linking purposes

find_package(CPlex)

list(INSERT CMAKE_PREFIX_PATH 0 "${NUTMEG_ROOT}" "$ENV{NUTMEG_ROOT}")

find_path(
  NUTMEG_INCLUDE Nutmeg/Nutmeg.h
  HINTS ${CMAKE_SOURCE_DIR}/..
)
#message("${NUTMEG_INCLUDE}")

find_library(
  NUTMEG_LIBRARY NAMES nutmeg libnutmeg
  HINTS $ENV{NUTMEG_LIBRARY_DIR} ${CMAKE_SOURCE_DIR}/../build
  PATH_SUFFIXES lib build
)
#message("${NUTMEG_LIBRARY}")
get_filename_component(NUTMEG_BUILD_DIR ${NUTMEG_LIBRARY} DIRECTORY)

find_library(
        FMT_LIBRARY NAMES fmt
        HINTS $ENV{NUTMEG_LIBRARY_DIR} ${CMAKE_SOURCE_DIR}/../build
        PATH_SUFFIXES lib build fmt
)

option(ZLIB "should zlib be linked" OFF)
option(GMP "should gmp be linked" OFF)
#set(USE_SCIP ON)
#set(SCIP_DIR "${NUTMEG_INCLUDE}/scipoptsuite-6.0.2/scip")
#message("${SCIP_DIR}")
find_library(
        NUTMEG_SCIP_LIBRARY NAMES scip
        HINTS ${NUTMEG_BUILD_DIR}
        PATH_SUFFIXES lib build
)
#message("${NUTMEG_SCIP_LIBRARY}")

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set NUTMEG_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Nutmeg
  FOUND_VAR NUTMEG_FOUND
  REQUIRED_VARS NUTMEG_INCLUDE NUTMEG_LIBRARY
  FAIL_MESSAGE "Could NOT find Nutmeg, use NUTMEG_ROOT to hint its location"
)

mark_as_advanced(NUTMEG_INCLUDE NUTMEG_LIBRARY)
list(REMOVE_AT CMAKE_PREFIX_PATH 1 0)

if(NUTMEG_FOUND)
  add_library(Nutmeg UNKNOWN IMPORTED)
  set_target_properties(Nutmeg PROPERTIES
    IMPORTED_LOCATION ${NUTMEG_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES "${NUTMEG_INCLUDE};$ENV{NUTMEG_LIBRARY_DIR};${NUTMEG_INCLUDE}/fmt/include;${NUTMEG_INCLUDE}/geas/include;${NUTMEG_INCLUDE}/scipoptsuite-6.0.2/scip/src/;${NUTMEG_BUILD_DIR}"
  )
endif()

set(NUTMEG_LIBRARIES ${NUTMEG_LIBRARY} ${GEAS_LIBRARY} ${NUTMEG_SCIP_LIBRARY})
#message("${NUTMEG_LIBRARIES}")
set(NUTMEG_INCLUDE_DIRS "${NUTMEG_INCLUDE}" "$ENV{NUTMEG_LIBRARY_DIR}" "${NUTMEG_INCLUDE}/fmt/include" "${NUTMEG_INCLUDE}/geas/include" "${NUTMEG_INCLUDE}/scipoptsuite-6.0.2/scip/src/" "${NUTMEG_BUILD_DIR}")

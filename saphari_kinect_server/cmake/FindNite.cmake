# - Try to find the Nite libraries
# Once done this will define
#
# Nite_FOUND - system has Nite
# Nite_INCLUDE_DIR - the Nite include directory
# Nite_LIBRARIES - Nite library

message(STATUS "Looking for Nite...")

find_path(Nite_INCLUDE_DIR
          NAMES XnVNite.h
          HINTS /usr/include/nite /usr/local/include/nite
)

find_library(Nite_LIBRARIES
             NAMES XnVNite_1_3_1
             HINTS /usr/lib /usr/local/lib)

if(Nite_INCLUDE_DIR AND Nite_LIBRARIES)
  set(Nite_FOUND 1)
        
  set(Nite_INCLUDE_DIR ${Nite_INCLUDE_DIR} CACHE PATH "Nite header directory" FORCE)
        
  if(NOT Nite_FIND_QUIETLY)
    message(STATUS "Found Nite: ${Nite_LIBRARIES}")
  endif(NOT Nite_FIND_QUIETLY)

else(Nite_INCLUDE_DIR AND Nite_LIBRARIES)

  set(Nite_FOUND 0 CACHE BOOL "Nite not found")
  if(Nite_FIND_REQUIRED)
    message(FATAL_ERROR "Nite not found, error")
  else()
    MESSAGE(STATUS "Nite not found, disabled")
  endif()
endif(Nite_INCLUDE_DIR AND Nite_LIBRARIES)

MARK_AS_ADVANCED(Nite_INCLUDE_DIR Nite_LIBRARIES)


# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_spraying_pathways_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED spraying_pathways_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(spraying_pathways_FOUND FALSE)
  elseif(NOT spraying_pathways_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(spraying_pathways_FOUND FALSE)
  endif()
  return()
endif()
set(_spraying_pathways_CONFIG_INCLUDED TRUE)

# output package information
if(NOT spraying_pathways_FIND_QUIETLY)
  message(STATUS "Found spraying_pathways: 0.0.0 (${spraying_pathways_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'spraying_pathways' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${spraying_pathways_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(spraying_pathways_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${spraying_pathways_DIR}/${_extra}")
endforeach()

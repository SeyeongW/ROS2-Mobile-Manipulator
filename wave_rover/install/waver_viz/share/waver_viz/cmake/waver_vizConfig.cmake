# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_waver_viz_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED waver_viz_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(waver_viz_FOUND FALSE)
  elseif(NOT waver_viz_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(waver_viz_FOUND FALSE)
  endif()
  return()
endif()
set(_waver_viz_CONFIG_INCLUDED TRUE)

# output package information
if(NOT waver_viz_FIND_QUIETLY)
  message(STATUS "Found waver_viz: 1.0.0 (${waver_viz_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'waver_viz' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${waver_viz_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(waver_viz_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${waver_viz_DIR}/${_extra}")
endforeach()

# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mpu9250_publisher_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mpu9250_publisher_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mpu9250_publisher_FOUND FALSE)
  elseif(NOT mpu9250_publisher_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mpu9250_publisher_FOUND FALSE)
  endif()
  return()
endif()
set(_mpu9250_publisher_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mpu9250_publisher_FIND_QUIETLY)
  message(STATUS "Found mpu9250_publisher: 0.0.0 (${mpu9250_publisher_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mpu9250_publisher' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mpu9250_publisher_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mpu9250_publisher_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mpu9250_publisher_DIR}/${_extra}")
endforeach()

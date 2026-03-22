#######################
# File: FindGrok.cmake
#######################

# Function to transform version numbers into a numerical format
function(transform_version _numerical_result _version_major _version_minor _version_patch)
    set(factor 100)
    if(_version_minor GREATER 99)
        set(factor 1000)
    endif()
    if(_version_patch GREATER 99)
        set(factor 1000)
    endif()
    math(EXPR _internal_numerical_result
         "${_version_major}*${factor}*${factor} + ${_version_minor}*${factor} + ${_version_patch}")
    set(${_numerical_result} ${_internal_numerical_result} PARENT_SCOPE)
endfunction()


###################################################
# - Find Grok
# Find Grok includes and libraries
#
# IMPORTED Target
#      GROK::Grok
#
# This module defines
#  GROK_INCLUDE_DIRS, where to find grok.h, etc.
#  GROK_LIBRARIES, the libraries needed to use Grok.
#  GROK_FOUND, If false, do not try to use Grok.
####################################################

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_GROK QUIET libgrokj2k)
    set(GROK_VERSION_STRING ${PC_GROK_VERSION})
endif()

# Set default Grok versions to search for
set(GROK_VERSIONS 20.2) # Add more versions if needed

# Locate Grok headers
foreach(VERSION ${GROK_VERSIONS})
    find_path(GROK_INCLUDE_DIRS grk_config.h
              PATH_SUFFIXES
                  grok-${VERSION}
              HINTS ${PC_GROK_INCLUDE_DIRS} $ENV{GROK_ROOT}/include
              DOC "Location of Grok Headers"
    )
    if(GROK_INCLUDE_DIRS)
        break()
    endif()
endforeach()

# Locate Grok libraries
find_library(GROK_LIBRARIES
             NAMES grokj2k
             HINTS ${PC_GROK_LIBRARY_DIRS} $ENV{GROK_ROOT}/lib
             DOC "Location of Grok Library"
)
mark_as_advanced(GROK_LIBRARIES GROK_INCLUDE_DIRS)

# If headers are found, determine Grok version
if(GROK_INCLUDE_DIR)
    if(DEFINED GROK_VERSION_STRING)
        string(REGEX MATCH "([0-9]+).([0-9]+).([0-9]+)" GRK_VERSION ${GROK_VERSION_STRING})
        if(GRK_VERSION)
            transform_version(GROK_VERSION_NUM ${CMAKE_MATCH_1} ${CMAKE_MATCH_2} ${CMAKE_MATCH_3})
        else()
            message(FATAL_ERROR "Grok version not found or invalid.")
        endif()
    endif()
endif()

# Handle standard package detection
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Grok
                                  FOUND_VAR GROK_FOUND
                                  REQUIRED_VARS GROK_LIBRARIES GROK_INCLUDE_DIRS
                                  VERSION_VAR GROK_VERSION_STRING)

# Configure imported target
if(GROK_FOUND)
    if(NOT TARGET GROK::Grok)
        add_library(GROK::Grok UNKNOWN IMPORTED)
        set_target_properties(GROK::Grok PROPERTIES
                              INTERFACE_INCLUDE_DIRECTORIES "${GROK_INCLUDE_DIRS}"
                              IMPORTED_LINK_INTERFACE_LANGUAGES "C"
                              IMPORTED_LOCATION "${GROK_LIBRARIES}")
    endif()
endif()

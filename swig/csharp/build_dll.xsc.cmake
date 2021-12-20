#
# script to build the C# DLLs using csc.exe or msc.exe
#

set(CSC_OPTIONS /unsafe /debug:full /target:library /out:${TARGET_SUBDIR}/${CSHARP_TARGET})
if(CSHARP_DEPENDS)
    foreach(_depends IN LISTS CSHARP_DEPENDS)
        list(APPEND CSC_OPTIONS /r:${_depends})
    endforeach ()
endif()
if(WIN32)
    list(APPEND CSC_OPTIONS /define:CLR4)
endif()
file(GLOB SOURCES "${BUILD_DIR}/${TARGET_SUBDIR}/*.cs")
list(APPEND SOURCES ${SOURCE_DIR}/AssemblyInfo.cs)
set(NATIVE_SOURCES)
foreach(_src IN LISTS SOURCES)
    file(TO_NATIVE_PATH "${_src}" _src)
    list(APPEND NATIVE_SOURCES "${_src}")
endforeach()
execute_process(
    COMMAND ${CSHARP_COMPILER} ${CSC_OPTIONS} ${NATIVE_SOURCES}
    # COMMAND_ECHO STDOUT
    WORKING_DIRECTORY ${BUILD_DIR})

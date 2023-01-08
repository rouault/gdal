add_library(
  libarchive_slim OBJECT
  archive_acl.c
  archive_check_magic.c
  archive_entry.c
  archive_entry_xattr.c
  archive_entry_sparse.c
  archive_ppmd7.c
  archive_read.c
  archive_read_support_format_7zip.c
  archive_string.c
  archive_string_sprintf.c
  archive_util.c
  archive_virtual.c
  archive_windows.c
)
set_property(TARGET libarchive_slim PROPERTY POSITION_INDEPENDENT_CODE ${GDAL_OBJECT_LIBRARIES_POSITION_INDEPENDENT_CODE})
target_sources(${GDAL_LIB_TARGET_NAME} PRIVATE $<TARGET_OBJECTS:libarchive_slim>)
target_compile_definitions(libarchive_slim PRIVATE "-DPLATFORM_CONFIG_H=\"archive_gdal_config.h\"" HAVE_STDINT_H HAVE_INTTYPES_H HAVE_LIMITS_H HAVE_STDLIB_H HAVE_STRING_H HAVE_WCHAR_H HAVE_ERRNO_H HAVE_WCSCPY HAVE_WCSLEN HAVE_DECL_SIZE_MAX HAVE_DECL_SSIZE_MAX HAVE_DECL_UINT32_MAX HAVE_DECL_INT32_MAX HAVE_DECL_INT32_MIN HAVE_DECL_UINT64_MAX HAVE_DECL_INT64_MAX HAVE_DECL_INT64_MIN HAVE_DECL_UINTMAX_MAX HAVE_DECL_INTMAX_MAX HAVE_DECL_INTMAX_MIN)

if(WIN32)
  if("${CMAKE_SIZEOF_VOID_P}" EQUAL 8)
    target_compile_definitions(libarchive_slim PRIVATE -Dssize_t=int64_t)
  else()
    target_compile_definitions(libarchive_slim PRIVATE -Dssize_t=long)
  endif()
  target_compile_definitions(libarchive_slim PRIVATE -Dpid_t=int)
  target_compile_definitions(libarchive_slim PRIVATE -Dgid_t=short)
  target_compile_definitions(libarchive_slim PRIVATE -Duid_t=short)
  target_compile_definitions(libarchive_slim PRIVATE -Dmode_t=uint16_t)
endif()

target_compile_definitions(libarchive_slim PRIVATE -DHAVE_ZLIB_H -DHAVE_ZLIB)
if (GDAL_USE_ZLIB_INTERNAL)
  gdal_add_vendored_lib(libarchive_slim libz)
endif ()

# Now external libraries

if (NOT GDAL_USE_ZLIB_INTERNAL)
  if (MSVC)
    target_compile_definitions(libarchive_slim PRIVATE -DZLIB_DLL)
  endif (MSVC)
  gdal_target_link_libraries(libarchive_slim PRIVATE ZLIB::ZLIB)
endif ()

if (GDAL_USE_LIBLZMA)
  target_compile_definitions(libarchive_slim PRIVATE HAVE_LZMA_H)
  gdal_target_link_libraries(libarchive_slim PRIVATE LibLZMA::LibLZMA)
endif ()

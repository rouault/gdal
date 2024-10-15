/******************************************************************************
 * Name:     gdal_typetraits.h
 * Project:  GDAL Core
 * Purpose:  Type traits for mapping C++ types to and from GDAL/OGR types.
 * Author:   Robin Princeley, <rprinceley at esri dot com>
 *
 ******************************************************************************
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 ****************************************************************************/
#if !defined(GDAL_TYPETRAITS_H_INCLUDED)
#define GDAL_TYPETRAITS_H_INCLUDED

#include "gdal_priv.h"

// NOTE: below GDAL_ENABLE_FLOAT16 is not guaranteed to be stable and is
// mostly for Esri internal needs for now. Might be revisited if/once RFC 100
// (https://github.com/OSGeo/gdal/pull/10146) is adopted.
#ifdef GDAL_ENABLE_FLOAT16
#if defined(__GNUC__) || defined(__clang__)
#define __STDC_WANT_IEC_60559_TYPES_EXT__
#include <float.h>  // Also brings in _Float16
#endif
#endif

#include <complex>

namespace gdal
{

/** Map a GDALDataType to the most suitable OGRFieldType.
 *
 * Note that GDT_UInt32 is mapped to OFTInteger64 to avoid data losses.
 * GDT_UInt64 is mapped to OFTReal, which can be lossy. If values are
 * guaranteed to be in [0, INT64_MAX] range, callers might want to use
 * OFTInteger64 instead.
 * There is no mapping for complex data types.
 *
 * @since 3.11
 */
constexpr inline OGRFieldType GetOGRFieldType(const GDALDataType gdal_type)
{
    switch (gdal_type)
    {
        case GDT_Byte:
        case GDT_Int8:
        case GDT_Int16:
        case GDT_Int32:
        case GDT_UInt16:
            return OFTInteger;
        case GDT_UInt32:
        case GDT_Int64:
            return OFTInteger64;
        case GDT_UInt64:  // Questionable
        case GDT_Float32:
        case GDT_Float64:
            return OFTReal;
        case GDT_CInt16:
        case GDT_CInt32:
        case GDT_CFloat32:
        case GDT_CFloat64:
        case GDT_Unknown:
        case GDT_TypeCount:
            break;
    }
    return OFTMaxType;
}

/** Map a GDALDataType to the most suitable OGRFieldSubType.
 *
 * @since 3.11
 */
constexpr inline OGRFieldSubType
GetOGRFieldSubType(const GDALDataType gdal_type)
{
    switch (gdal_type)
    {
        case GDT_Byte:
            break;
        case GDT_Int8:
            break;
        case GDT_Int16:
            return OFSTInt16;
        case GDT_Int32:
            break;
        case GDT_UInt16:
            break;
        case GDT_UInt32:
            break;
        case GDT_Int64:
            break;
        case GDT_UInt64:
            break;
        case GDT_Float32:
            return OFSTFloat32;
        case GDT_Float64:
            break;
        case GDT_CInt16:
            break;
        case GDT_CInt32:
            break;
        case GDT_CFloat32:
            break;
        case GDT_CFloat64:
            break;
        case GDT_Unknown:
            break;
        case GDT_TypeCount:
            break;
    }
    return OFSTNone;
}

/** Trait accepting a C++ type ([u]int[8/16/32/64], float, double,
 * std::complex<float>, std::complex<double> or std::string)
 * and mapping it to GDALDataType / OGRFieldType.
 *
 * Each specialization has the following members:
 * static constexpr GDALDataType gdal_type;
 * static constexpr size_t size;
 * static constexpr OGRFieldType ogr_type;
 * static constexpr OGRFieldSubType ogr_subtype;
 *
 * @since 3.11
 */
template <typename T> struct CXXTypeTraits
{
};

namespace detail
{
template <typename T> struct CXXTypeTraitsBase
{
    static constexpr GDALDataType gdal_type = GDT_Unknown;
    static constexpr size_t size = sizeof(T);
    static constexpr OGRFieldType ogr_type =
        GetOGRFieldType(CXXTypeTraits<T>::gdal_type);
    static constexpr OGRFieldSubType ogr_subtype =
        GetOGRFieldSubType(CXXTypeTraits<T>::gdal_type);

    static inline GDALExtendedDataType GetExtendedDataType()
    {
        return GDALExtendedDataType::Create(CXXTypeTraits<T>::gdal_type);
    }
};
}  // namespace detail

//! @cond Doxygen_Suppress
template <>
struct CXXTypeTraits<int8_t> : public detail::CXXTypeTraitsBase<int8_t>
{
    static constexpr GDALDataType gdal_type = GDT_Int8;
};

template <>
struct CXXTypeTraits<uint8_t> : public detail::CXXTypeTraitsBase<uint8_t>
{
    static constexpr GDALDataType gdal_type = GDT_Byte;
};

template <>
struct CXXTypeTraits<int16_t> : public detail::CXXTypeTraitsBase<int16_t>
{
    static constexpr GDALDataType gdal_type = GDT_Int16;
};

template <>
struct CXXTypeTraits<uint16_t> : public detail::CXXTypeTraitsBase<uint16_t>
{
    static constexpr GDALDataType gdal_type = GDT_UInt16;
};

template <>
struct CXXTypeTraits<int32_t> : public detail::CXXTypeTraitsBase<int32_t>
{
    static constexpr GDALDataType gdal_type = GDT_Int32;
};

template <>
struct CXXTypeTraits<uint32_t> : public detail::CXXTypeTraitsBase<uint32_t>
{
    static constexpr GDALDataType gdal_type = GDT_UInt32;
};

template <>
struct CXXTypeTraits<int64_t> : public detail::CXXTypeTraitsBase<int64_t>
{
    static constexpr GDALDataType gdal_type = GDT_Int64;
};

template <>
struct CXXTypeTraits<uint64_t> : public detail::CXXTypeTraitsBase<uint64_t>
{
    static constexpr GDALDataType gdal_type = GDT_UInt64;
};

template <>
struct CXXTypeTraits<float> : public detail::CXXTypeTraitsBase<float>
{
    static constexpr GDALDataType gdal_type = GDT_Float32;
};

template <>
struct CXXTypeTraits<double> : public detail::CXXTypeTraitsBase<double>
{
    static constexpr GDALDataType gdal_type = GDT_Float64;
};

template <>
struct CXXTypeTraits<std::complex<float>>
    : public detail::CXXTypeTraitsBase<std::complex<float>>
{
    static constexpr GDALDataType gdal_type = GDT_CFloat32;
};

template <>
struct CXXTypeTraits<std::complex<double>>
    : public detail::CXXTypeTraitsBase<std::complex<double>>
{
    static constexpr GDALDataType gdal_type = GDT_CFloat64;
};

#if defined(GDAL_ENABLE_FLOAT16) && defined(FLT16_MAX) && defined(FLT16_MIN)
template <> struct CXXTypeTraits<_Float16>
{
    static constexpr GDALDataType gdal_type = GDT_Float16;
};
#endif

template <>
struct CXXTypeTraits<std::string>
    : public detail::CXXTypeTraitsBase<std::string>
{
    static constexpr size_t size = 0;
    static constexpr OGRFieldType ogr_type = OFTString;

    static inline GDALExtendedDataType GetExtendedDataType()
    {
        return GDALExtendedDataType::CreateString();
    }
};

//! @endcond

/** Trait accepting a GDALDataType and mapping it to corresponding C++ type and
 * OGRFieldType
 *
 * Each specialization has the following members:
 * typedef T type; (except for GDT_CInt16 and GDT_CInt32)
 * static constexpr size_t size;
 * static constexpr OGRFieldType ogr_type;
 * static constexpr OGRFieldSubType ogr_subtype;
 * static inline GDALExtendedDataType GetExtendedDataType();
 *
 * @since 3.11
 */
template <GDALDataType T> struct GDALDataTypeTraits
{
};

namespace detail
{
template <GDALDataType T> struct GDALDataTypeTraitsBase
{

    static constexpr size_t size = sizeof(typename GDALDataTypeTraits<T>::type);
    static constexpr OGRFieldType ogr_type = GetOGRFieldType(T);
    static constexpr OGRFieldSubType ogr_subtype = GetOGRFieldSubType(T);

    static inline GDALExtendedDataType GetExtendedDataType()
    {
        return GDALExtendedDataType::Create(T);
    }
};
}  // namespace detail

//! @cond Doxygen_Suppress
template <>
struct GDALDataTypeTraits<GDT_Int8>
    : public detail::GDALDataTypeTraitsBase<GDT_Int8>
{
    typedef int8_t type;
};

template <>
struct GDALDataTypeTraits<GDT_Byte>
    : public detail::GDALDataTypeTraitsBase<GDT_Byte>
{
    typedef uint8_t type;
};

template <>
struct GDALDataTypeTraits<GDT_Int16>
    : public detail::GDALDataTypeTraitsBase<GDT_Int16>
{
    typedef int16_t type;
};

template <>
struct GDALDataTypeTraits<GDT_UInt16>
    : public detail::GDALDataTypeTraitsBase<GDT_UInt16>
{
    typedef uint16_t type;
};

template <>
struct GDALDataTypeTraits<GDT_Int32>
    : public detail::GDALDataTypeTraitsBase<GDT_Int32>
{
    typedef int32_t type;
};

template <>
struct GDALDataTypeTraits<GDT_UInt32>
    : public detail::GDALDataTypeTraitsBase<GDT_UInt32>
{
    typedef uint32_t type;
};

template <>
struct GDALDataTypeTraits<GDT_Int64>
    : public detail::GDALDataTypeTraitsBase<GDT_Int64>
{
    typedef int64_t type;
};

template <>
struct GDALDataTypeTraits<GDT_UInt64>
    : public detail::GDALDataTypeTraitsBase<GDT_UInt64>
{
    typedef uint64_t type;
};

template <>
struct GDALDataTypeTraits<GDT_Float32>
    : public detail::GDALDataTypeTraitsBase<GDT_Float32>
{
    typedef float type;
};

template <>
struct GDALDataTypeTraits<GDT_Float64>
    : public detail::GDALDataTypeTraitsBase<GDT_Float64>
{
    typedef double type;
};

template <>
struct GDALDataTypeTraits<GDT_CInt16>
    : public detail::GDALDataTypeTraitsBase<GDT_CInt16>
{
    // typedef type not available !
    static constexpr size_t size = sizeof(int16_t) * 2;
};

template <>
struct GDALDataTypeTraits<GDT_CInt32>
    : public detail::GDALDataTypeTraitsBase<GDT_CInt32>
{
    // typedef type not available !
    static constexpr size_t size = sizeof(int32_t) * 2;
};

template <>
struct GDALDataTypeTraits<GDT_CFloat32>
    : public detail::GDALDataTypeTraitsBase<GDT_CFloat32>
{
    typedef std::complex<float> type;
};

template <>
struct GDALDataTypeTraits<GDT_CFloat64>
    : public detail::GDALDataTypeTraitsBase<GDT_CFloat64>
{
    typedef std::complex<double> type;
};

//! @endcond

/** Map a GDALExtendedDataType to the most suitable OGRFieldType.
 *
 * Note that GDT_UInt32 is mapped to OFTInteger64 to avoid data losses.
 * GDT_UInt64 is mapped to OFTReal, which can be lossy. If values are
 * guaranteed to be in [0, INT64_MAX] range, callers might want to use
 * OFTInteger64 instead.
 *
 * @since 3.11
 */
inline OGRFieldType GetOGRFieldType(const GDALExtendedDataType &oEDT)
{
    if (oEDT.GetClass() == GEDTC_NUMERIC)
        return GetOGRFieldType(oEDT.GetNumericDataType());
    else if (oEDT.GetClass() == GEDTC_STRING)
        return OFTString;
    return OFTMaxType;
}

}  // namespace gdal

#endif  // GDAL_TYPETRAITS_H_INCLUDED

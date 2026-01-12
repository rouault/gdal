/******************************************************************************
 *
 * Project:  GDAL
 * Purpose:  Zarr driver
 * Author:   Even Rouault <even dot rouault at spatialys.com>
 *
 ******************************************************************************
 * Copyright (c) 2023, Even Rouault <even dot rouault at spatialys.com>
 *
 * SPDX-License-Identifier: MIT
 ****************************************************************************/

#include "zarr_v3_codec.h"

#include "cpl_compressor.h"
#include "crc32c.h"

/************************************************************************/
/*                          ZarrV3Codec()                               */
/************************************************************************/

ZarrV3Codec::ZarrV3Codec(const std::string &osName) : m_osName(osName)
{
}

/************************************************************************/
/*                         ~ZarrV3Codec()                               */
/************************************************************************/

ZarrV3Codec::~ZarrV3Codec() = default;

/************************************************************************/
/*                    ZarrV3Codec::DecodePartial()                      */
/************************************************************************/

bool ZarrV3Codec::DecodePartial(VSIVirtualHandle *,
                                const ZarrByteVectorQuickResize &,
                                ZarrByteVectorQuickResize &,
                                std::vector<size_t> &, std::vector<size_t> &)
{
    // Normally we should not hit that...
    CPLError(CE_Failure, CPLE_NotSupported,
             "Codec %s does not support partial decoding", m_osName.c_str());
    return false;
}

/************************************************************************/
/*                      ZarrV3CodecAbstractCompressor()                 */
/************************************************************************/

ZarrV3CodecAbstractCompressor::ZarrV3CodecAbstractCompressor(
    const std::string &osName)
    : ZarrV3Codec(osName)
{
}

/************************************************************************/
/*                 ZarrV3CodecAbstractCompressor::Encode()              */
/************************************************************************/

bool ZarrV3CodecAbstractCompressor::Encode(
    const ZarrByteVectorQuickResize &abySrc,
    ZarrByteVectorQuickResize &abyDst) const
{
    abyDst.resize(abyDst.capacity());
    void *pOutputData = abyDst.data();
    size_t nOutputSize = abyDst.size();
    bool bRet = m_pCompressor->pfnFunc(
        abySrc.data(), abySrc.size(), &pOutputData, &nOutputSize,
        m_aosCompressorOptions.List(), m_pCompressor->user_data);
    if (bRet)
    {
        abyDst.resize(nOutputSize);
    }
    else if (nOutputSize > abyDst.size())
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "%s codec:Encode(): output buffer too small",
                 m_osName.c_str());
    }
    return bRet;
}

/************************************************************************/
/*                 ZarrV3CodecAbstractCompressor::Decode()              */
/************************************************************************/

bool ZarrV3CodecAbstractCompressor::Decode(
    const ZarrByteVectorQuickResize &abySrc,
    ZarrByteVectorQuickResize &abyDst) const
{
    abyDst.resize(abyDst.capacity());
    void *pOutputData = abyDst.data();
    size_t nOutputSize = abyDst.size();
    bool bRet = m_pDecompressor->pfnFunc(abySrc.data(), abySrc.size(),
                                         &pOutputData, &nOutputSize, nullptr,
                                         m_pDecompressor->user_data);
    if (bRet)
    {
        abyDst.resize(nOutputSize);
    }
    else if (nOutputSize > abyDst.size())
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "%s codec:Decode(): output buffer too small",
                 m_osName.c_str());
    }
    return bRet;
}

/************************************************************************/
/*                        ZarrV3CodecGZip()                             */
/************************************************************************/

ZarrV3CodecGZip::ZarrV3CodecGZip() : ZarrV3CodecAbstractCompressor(NAME)
{
}

/************************************************************************/
/*                           GetConfiguration()                         */
/************************************************************************/

/* static */ CPLJSONObject ZarrV3CodecGZip::GetConfiguration(int nLevel)
{
    CPLJSONObject oConfig;
    oConfig.Add("level", nLevel);
    return oConfig;
}

/************************************************************************/
/*                   ZarrV3CodecGZip::InitFromConfiguration()           */
/************************************************************************/

bool ZarrV3CodecGZip::InitFromConfiguration(
    const CPLJSONObject &configuration,
    const ZarrArrayMetadata &oInputArrayMetadata,
    ZarrArrayMetadata &oOutputArrayMetadata)
{
    m_pCompressor = CPLGetCompressor("gzip");
    m_pDecompressor = CPLGetDecompressor("gzip");
    if (!m_pCompressor || !m_pDecompressor)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "gzip compressor not available");
        return false;
    }

    m_oConfiguration = configuration.Clone();
    m_oInputArrayMetadata = oInputArrayMetadata;
    // byte->byte codec
    oOutputArrayMetadata = oInputArrayMetadata;

    int nLevel = 6;

    if (configuration.IsValid())
    {
        if (configuration.GetType() != CPLJSONObject::Type::Object)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec gzip: configuration is not an object");
            return false;
        }

        for (const auto &oChild : configuration.GetChildren())
        {
            if (oChild.GetName() != "level")
            {
                CPLError(
                    CE_Failure, CPLE_AppDefined,
                    "Codec gzip: configuration contains a unhandled member: %s",
                    oChild.GetName().c_str());
                return false;
            }
        }

        const auto oLevel = configuration.GetObj("level");
        if (oLevel.IsValid())
        {
            if (oLevel.GetType() != CPLJSONObject::Type::Integer)
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Codec gzip: level is not an integer");
                return false;
            }
            nLevel = oLevel.ToInteger();
            if (nLevel < 0 || nLevel > 9)
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Codec gzip: invalid value for level: %d", nLevel);
                return false;
            }
        }
    }

    m_aosCompressorOptions.SetNameValue("LEVEL", CPLSPrintf("%d", nLevel));

    return true;
}

/************************************************************************/
/*                      ZarrV3CodecGZip::Clone()                        */
/************************************************************************/

std::unique_ptr<ZarrV3Codec> ZarrV3CodecGZip::Clone() const
{
    auto psClone = std::make_unique<ZarrV3CodecGZip>();
    ZarrArrayMetadata oOutputArrayMetadata;
    psClone->InitFromConfiguration(m_oConfiguration, m_oInputArrayMetadata,
                                   oOutputArrayMetadata);
    return psClone;
}

/************************************************************************/
/*                        ZarrV3CodecZstd()                             */
/************************************************************************/

ZarrV3CodecZstd::ZarrV3CodecZstd() : ZarrV3CodecAbstractCompressor(NAME)
{
}

/************************************************************************/
/*                           GetConfiguration()                         */
/************************************************************************/

/* static */ CPLJSONObject ZarrV3CodecZstd::GetConfiguration(int nLevel,
                                                             bool checksum)
{
    CPLJSONObject oConfig;
    oConfig.Add("level", nLevel);
    oConfig.Add("checksum", checksum);
    return oConfig;
}

/************************************************************************/
/*                   ZarrV3CodecZstd::InitFromConfiguration()           */
/************************************************************************/

bool ZarrV3CodecZstd::InitFromConfiguration(
    const CPLJSONObject &configuration,
    const ZarrArrayMetadata &oInputArrayMetadata,
    ZarrArrayMetadata &oOutputArrayMetadata)
{
    m_pCompressor = CPLGetCompressor("zstd");
    m_pDecompressor = CPLGetDecompressor("zstd");
    if (!m_pCompressor || !m_pDecompressor)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "zstd compressor not available");
        return false;
    }

    m_oConfiguration = configuration.Clone();
    m_oInputArrayMetadata = oInputArrayMetadata;
    // byte->byte codec
    oOutputArrayMetadata = oInputArrayMetadata;

    int nLevel = 13;
    bool bChecksum = false;

    if (configuration.IsValid())
    {
        if (configuration.GetType() != CPLJSONObject::Type::Object)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec zstd: configuration is not an object");
            return false;
        }

        for (const auto &oChild : configuration.GetChildren())
        {
            if (oChild.GetName() != "level" && oChild.GetName() != "checksum")
            {
                CPLError(
                    CE_Failure, CPLE_AppDefined,
                    "Codec zstd: configuration contains a unhandled member: %s",
                    oChild.GetName().c_str());
                return false;
            }
        }

        const auto oLevel = configuration.GetObj("level");
        if (oLevel.IsValid())
        {
            if (oLevel.GetType() != CPLJSONObject::Type::Integer)
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Codec zstd: level is not an integer");
                return false;
            }
            nLevel = oLevel.ToInteger();
            if (nLevel < 0 || nLevel > 22)
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Codec zstd: invalid value for level: %d", nLevel);
                return false;
            }
        }

        const auto oChecksum = configuration.GetObj("checksum");
        if (oChecksum.IsValid())
        {
            if (oChecksum.GetType() != CPLJSONObject::Type::Boolean)
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Codec zstd: checksum is not a boolean");
                return false;
            }
            bChecksum = oChecksum.ToBool();
        }
    }

    m_aosCompressorOptions.SetNameValue("LEVEL", CPLSPrintf("%d", nLevel));
    if (bChecksum)
        m_aosCompressorOptions.SetNameValue("CHECKSUM", "YES");

    return true;
}

/************************************************************************/
/*                      ZarrV3CodecZstd::Clone()                        */
/************************************************************************/

std::unique_ptr<ZarrV3Codec> ZarrV3CodecZstd::Clone() const
{
    auto psClone = std::make_unique<ZarrV3CodecZstd>();
    ZarrArrayMetadata oOutputArrayMetadata;
    psClone->InitFromConfiguration(m_oConfiguration, m_oInputArrayMetadata,
                                   oOutputArrayMetadata);
    return psClone;
}

/************************************************************************/
/*                       ZarrV3CodecBlosc()                             */
/************************************************************************/

ZarrV3CodecBlosc::ZarrV3CodecBlosc() : ZarrV3CodecAbstractCompressor(NAME)
{
}

/************************************************************************/
/*                           GetConfiguration()                         */
/************************************************************************/

/* static */ CPLJSONObject
ZarrV3CodecBlosc::GetConfiguration(const char *cname, int clevel,
                                   const char *shuffle, int typesize,
                                   int blocksize)
{
    CPLJSONObject oConfig;
    oConfig.Add("cname", cname);
    oConfig.Add("clevel", clevel);
    oConfig.Add("shuffle", shuffle);
    if (strcmp(shuffle, "noshuffle") != 0)
        oConfig.Add("typesize", typesize);
    oConfig.Add("blocksize", blocksize);
    return oConfig;
}

/************************************************************************/
/*                   ZarrV3CodecBlosc::InitFromConfiguration()           */
/************************************************************************/

bool ZarrV3CodecBlosc::InitFromConfiguration(
    const CPLJSONObject &configuration,
    const ZarrArrayMetadata &oInputArrayMetadata,
    ZarrArrayMetadata &oOutputArrayMetadata)
{
    m_pCompressor = CPLGetCompressor("blosc");
    m_pDecompressor = CPLGetDecompressor("blosc");
    if (!m_pCompressor || !m_pDecompressor)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "blosc compressor not available");
        return false;
    }

    m_oConfiguration = configuration.Clone();
    m_oInputArrayMetadata = oInputArrayMetadata;
    // byte->byte codec
    oOutputArrayMetadata = oInputArrayMetadata;

    if (!configuration.IsValid() ||
        configuration.GetType() != CPLJSONObject::Type::Object)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Codec blosc: configuration missing or not an object");
        return false;
    }

    for (const auto &oChild : configuration.GetChildren())
    {
        const auto osName = oChild.GetName();
        if (osName != "cname" && osName != "clevel" && osName != "shuffle" &&
            osName != "typesize" && osName != "blocksize")
        {
            CPLError(
                CE_Failure, CPLE_AppDefined,
                "Codec blosc: configuration contains a unhandled member: %s",
                osName.c_str());
            return false;
        }
    }

    const auto oCname = configuration.GetObj("cname");
    if (oCname.GetType() != CPLJSONObject::Type::String)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Codec blosc: cname is missing or not a string");
        return false;
    }
    m_aosCompressorOptions.SetNameValue("CNAME", oCname.ToString().c_str());

    const auto oLevel = configuration.GetObj("clevel");
    if (oLevel.IsValid())
    {
        if (oLevel.GetType() != CPLJSONObject::Type::Integer)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec blosc: clevel is not an integer");
            return false;
        }
        const int nLevel = oLevel.ToInteger();
        if (nLevel < 0 || nLevel > 9)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec blosc: clevel value for level: %d", nLevel);
            return false;
        }
        m_aosCompressorOptions.SetNameValue("CLEVEL", CPLSPrintf("%d", nLevel));
    }

    const auto oShuffle = configuration.GetObj("shuffle");
    if (oShuffle.GetType() != CPLJSONObject::Type::String)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Codec blosc: shuffle is missing or not a string");
        return false;
    }
    if (oShuffle.ToString() == "noshuffle")
        m_aosCompressorOptions.SetNameValue("SHUFFLE", "NONE");
    else if (oShuffle.ToString() == "shuffle")
        m_aosCompressorOptions.SetNameValue("SHUFFLE", "BYTE");
    else if (oShuffle.ToString() == "bitshuffle")
        m_aosCompressorOptions.SetNameValue("SHUFFLE", "BIT");
    else
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Codec blosc: Invalid value for shuffle");
        return false;
    }

    const auto oTypesize = configuration.GetObj("typesize");
    if (oTypesize.IsValid())
    {
        if (oTypesize.GetType() != CPLJSONObject::Type::Integer)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec blosc: typesize is not an integer");
            return false;
        }
        const int nTypeSize = oTypesize.ToInteger();
        m_aosCompressorOptions.SetNameValue("TYPESIZE",
                                            CPLSPrintf("%d", nTypeSize));
    }

    const auto oBlocksize = configuration.GetObj("blocksize");
    if (oBlocksize.IsValid())
    {
        if (oBlocksize.GetType() != CPLJSONObject::Type::Integer)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec blosc: blocksize is not an integer");
            return false;
        }
        const int nBlocksize = oBlocksize.ToInteger();
        m_aosCompressorOptions.SetNameValue("BLOCKSIZE",
                                            CPLSPrintf("%d", nBlocksize));
    }

    return true;
}

/************************************************************************/
/*                      ZarrV3CodecBlosc::Clone()                        */
/************************************************************************/

std::unique_ptr<ZarrV3Codec> ZarrV3CodecBlosc::Clone() const
{
    auto psClone = std::make_unique<ZarrV3CodecBlosc>();
    ZarrArrayMetadata oOutputArrayMetadata;
    psClone->InitFromConfiguration(m_oConfiguration, m_oInputArrayMetadata,
                                   oOutputArrayMetadata);
    return psClone;
}

/************************************************************************/
/*                       ZarrV3CodecBytes()                            */
/************************************************************************/

ZarrV3CodecBytes::ZarrV3CodecBytes() : ZarrV3Codec(NAME)
{
}

/************************************************************************/
/*                           GetConfiguration()                         */
/************************************************************************/

/* static */ CPLJSONObject ZarrV3CodecBytes::GetConfiguration(bool bLittle)
{
    CPLJSONObject oConfig;
    oConfig.Add("endian", bLittle ? "little" : "big");
    return oConfig;
}

/************************************************************************/
/*                 ZarrV3CodecBytes::InitFromConfiguration()            */
/************************************************************************/

bool ZarrV3CodecBytes::InitFromConfiguration(
    const CPLJSONObject &configuration,
    const ZarrArrayMetadata &oInputArrayMetadata,
    ZarrArrayMetadata &oOutputArrayMetadata)
{
    m_oConfiguration = configuration.Clone();
    m_bLittle = true;
    m_oInputArrayMetadata = oInputArrayMetadata;
    oOutputArrayMetadata = oInputArrayMetadata;

    if (configuration.IsValid())
    {
        if (configuration.GetType() != CPLJSONObject::Type::Object)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec endian: configuration is not an object");
            return false;
        }

        for (const auto &oChild : configuration.GetChildren())
        {
            if (oChild.GetName() != "endian")
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Codec endian: configuration contains a unhandled "
                         "member: %s",
                         oChild.GetName().c_str());
                return false;
            }
        }

        const auto oEndian = configuration.GetObj("endian");
        if (oEndian.IsValid())
        {
            if (oEndian.GetType() != CPLJSONObject::Type::String)
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Codec gzip: endian is not a string");
                return false;
            }
            if (oEndian.ToString() == "little")
                m_bLittle = true;
            else if (oEndian.ToString() == "big")
                m_bLittle = false;
            else
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Codec gzip: invalid value for endian");
                return false;
            }
        }
    }

    return true;
}

/************************************************************************/
/*                     ZarrV3CodecBytes::Clone()                        */
/************************************************************************/

std::unique_ptr<ZarrV3Codec> ZarrV3CodecBytes::Clone() const
{
    auto psClone = std::make_unique<ZarrV3CodecBytes>();
    ZarrArrayMetadata oOutputArrayMetadata;
    psClone->InitFromConfiguration(m_oConfiguration, m_oInputArrayMetadata,
                                   oOutputArrayMetadata);
    return psClone;
}

/************************************************************************/
/*                      ZarrV3CodecBytes::Encode()                      */
/************************************************************************/

bool ZarrV3CodecBytes::Encode(const ZarrByteVectorQuickResize &abySrc,
                              ZarrByteVectorQuickResize &abyDst) const
{
    CPLAssert(!IsNoOp());

    size_t nEltCount = MultiplyElements(m_oInputArrayMetadata.anBlockSizes);
    size_t nNativeSize = m_oInputArrayMetadata.oElt.nativeSize;
    if (abySrc.size() < nEltCount * nNativeSize)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "ZarrV3CodecBytes::Encode(): input buffer too small");
        return false;
    }
    CPLAssert(abySrc.size() >= nEltCount * nNativeSize);
    abyDst.resize(nEltCount * nNativeSize);

    const GByte *pabySrc = abySrc.data();
    GByte *pabyDst = abyDst.data();

    if (m_oInputArrayMetadata.oElt.nativeType ==
        DtypeElt::NativeType::COMPLEX_IEEEFP)
    {
        nEltCount *= 2;
        nNativeSize /= 2;
    }
    if (nNativeSize == 2)
    {
        for (size_t i = 0; i < nEltCount; ++i)
        {
            const uint16_t val = CPL_SWAP16(*reinterpret_cast<const uint16_t *>(
                pabySrc + sizeof(uint16_t) * i));
            memcpy(pabyDst + sizeof(uint16_t) * i, &val, sizeof(val));
        }
    }
    else if (nNativeSize == 4)
    {
        for (size_t i = 0; i < nEltCount; ++i)
        {
            const uint32_t val = CPL_SWAP32(*reinterpret_cast<const uint32_t *>(
                pabySrc + sizeof(uint32_t) * i));
            memcpy(pabyDst + sizeof(uint32_t) * i, &val, sizeof(val));
        }
    }
    else if (nNativeSize == 8)
    {
        for (size_t i = 0; i < nEltCount; ++i)
        {
            const uint64_t val = CPL_SWAP64(*reinterpret_cast<const uint64_t *>(
                pabySrc + sizeof(uint64_t) * i));
            memcpy(pabyDst + sizeof(uint64_t) * i, &val, sizeof(val));
        }
    }
    else
    {
        CPLAssert(false);
    }
    return true;
}

/************************************************************************/
/*                      ZarrV3CodecBytes::Decode()                      */
/************************************************************************/

bool ZarrV3CodecBytes::Decode(const ZarrByteVectorQuickResize &abySrc,
                              ZarrByteVectorQuickResize &abyDst) const
{
    return Encode(abySrc, abyDst);
}

/************************************************************************/
/*                       ZarrV3CodecTranspose()                         */
/************************************************************************/

ZarrV3CodecTranspose::ZarrV3CodecTranspose() : ZarrV3Codec(NAME)
{
}

/************************************************************************/
/*                             IsNoOp()                                 */
/************************************************************************/

bool ZarrV3CodecTranspose::IsNoOp() const
{
    for (int i = 0; i < static_cast<int>(m_anOrder.size()); ++i)
    {
        if (m_anOrder[i] != i)
            return false;
    }
    return true;
}

/************************************************************************/
/*                           GetConfiguration()                         */
/************************************************************************/

/* static */ CPLJSONObject
ZarrV3CodecTranspose::GetConfiguration(const std::vector<int> &anOrder)
{
    CPLJSONObject oConfig;
    CPLJSONArray oOrder;
    for (const auto nVal : anOrder)
        oOrder.Add(nVal);
    oConfig.Add("order", oOrder);
    return oConfig;
}

/************************************************************************/
/*                ZarrV3CodecTranspose::InitFromConfiguration()         */
/************************************************************************/

bool ZarrV3CodecTranspose::InitFromConfiguration(
    const CPLJSONObject &configuration,
    const ZarrArrayMetadata &oInputArrayMetadata,
    ZarrArrayMetadata &oOutputArrayMetadata)
{
    m_oConfiguration = configuration.Clone();
    m_oInputArrayMetadata = oInputArrayMetadata;
    oOutputArrayMetadata = oInputArrayMetadata;

    if (!configuration.IsValid() &&
        configuration.GetType() != CPLJSONObject::Type::Object)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Codec transpose: configuration missing or not an object");
        return false;
    }

    for (const auto &oChild : configuration.GetChildren())
    {
        if (oChild.GetName() != "order")
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec transpose: configuration contains a unhandled "
                     "member: %s",
                     oChild.GetName().c_str());
            return false;
        }
    }

    const auto oOrder = configuration.GetObj("order");
    const int nDims = static_cast<int>(oInputArrayMetadata.anBlockSizes.size());
    if (oOrder.GetType() == CPLJSONObject::Type::String)
    {
        // Deprecated
        const auto osOrder = oOrder.ToString();
        if (osOrder == "C")
        {
            for (int i = 0; i < nDims; ++i)
            {
                m_anOrder.push_back(i);
            }
        }
        else if (osOrder == "F")
        {
            for (int i = 0; i < nDims; ++i)
            {
                m_anOrder.push_back(nDims - 1 - i);
                oOutputArrayMetadata.anBlockSizes[i] =
                    oInputArrayMetadata.anBlockSizes[nDims - 1 - i];
            }
        }
        else
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec transpose: invalid value for order");
            return false;
        }
    }
    else if (oOrder.GetType() == CPLJSONObject::Type::Array)
    {
        const auto oOrderArray = oOrder.ToArray();
        if (oOrderArray.Size() != nDims)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Codec transpose: order[] does not have the expected "
                     "number of elements");
            return false;
        }
        std::vector<int> oSet(nDims);
        oOutputArrayMetadata.anBlockSizes.clear();
        for (const auto &oVal : oOrderArray)
        {
            const int nVal = oVal.ToInteger();
            if (nVal < 0 || nVal >= nDims || oSet[nVal])
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Codec transpose: order[] does not define a valid "
                         "transposition");
                return false;
            }
            oSet[nVal] = true;
            m_anOrder.push_back(nVal);
            oOutputArrayMetadata.anBlockSizes.push_back(
                oInputArrayMetadata.anBlockSizes[nVal]);
        }
    }
    else
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Codec transpose: invalid value for order");
        return false;
    }

    int i = 0;
    m_anReverseOrder.resize(m_anOrder.size());
    for (const auto nVal : m_anOrder)
    {
        m_anReverseOrder[nVal] = i;
        ++i;
    }

    return true;
}

/************************************************************************/
/*              ZarrV3CodecTranspose::GetInnerMostBlockSize()           */
/************************************************************************/

std::vector<size_t> ZarrV3CodecTranspose::GetInnerMostBlockSize(
    const std::vector<size_t> &anInnerBlockSize) const
{
    std::vector<size_t> ret;
    for (int idx : m_anReverseOrder)
        ret.push_back(anInnerBlockSize[idx]);
    return ret;
}

/************************************************************************/
/*                   ZarrV3CodecTranspose::Clone()                      */
/************************************************************************/

std::unique_ptr<ZarrV3Codec> ZarrV3CodecTranspose::Clone() const
{
    auto psClone = std::make_unique<ZarrV3CodecTranspose>();
    ZarrArrayMetadata oOutputArrayMetadata;
    psClone->InitFromConfiguration(m_oConfiguration, m_oInputArrayMetadata,
                                   oOutputArrayMetadata);
    return psClone;
}

/************************************************************************/
/*                  ZarrV3CodecTranspose::Transpose()                   */
/************************************************************************/

bool ZarrV3CodecTranspose::Transpose(
    const ZarrByteVectorQuickResize &abySrc, ZarrByteVectorQuickResize &abyDst,
    bool bEncodeDirection, const std::vector<size_t> &anForwardBlockSizes) const
{
    CPLAssert(m_anOrder.size() == anForwardBlockSizes.size());
    CPLAssert(m_anReverseOrder.size() == anForwardBlockSizes.size());
    const size_t nDims = m_anOrder.size();
    const size_t nSourceSize = m_oInputArrayMetadata.oElt.nativeSize;
    CPLAssert(nDims > 0);
    if (abySrc.size() < MultiplyElements(anForwardBlockSizes) * nSourceSize)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "ZarrV3CodecTranspose::Transpose(): input buffer too small");
        return false;
    }
    abyDst.resize(MultiplyElements(anForwardBlockSizes) * nSourceSize);

    struct Stack
    {
        size_t nIters = 0;
        const GByte *src_ptr = nullptr;
        GByte *dst_ptr = nullptr;
        size_t src_inc_offset = 0;
        size_t dst_inc_offset = 0;
    };

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#endif
    std::vector<Stack> stack(nDims);
    stack.emplace_back(
        Stack());  // to make gcc 9.3 -O2 -Wnull-dereference happy
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

    if (!bEncodeDirection)
    {
        stack[m_anReverseOrder[nDims - 1]].src_inc_offset = nSourceSize;
        size_t nStride = nSourceSize;
        for (size_t i = nDims - 1; i > 0;)
        {
            --i;
            nStride *= anForwardBlockSizes[m_anReverseOrder[i + 1]];
            stack[m_anReverseOrder[i]].src_inc_offset = nStride;
        }

        stack[nDims - 1].dst_inc_offset = nSourceSize;
        nStride = nSourceSize;
        for (size_t i = nDims - 1; i > 0;)
        {
            --i;
            nStride *= anForwardBlockSizes[i + 1];
            stack[i].dst_inc_offset = nStride;
        }
    }
    else
    {
        stack[m_anReverseOrder[nDims - 1]].dst_inc_offset = nSourceSize;
        size_t nStride = nSourceSize;
        for (size_t i = nDims - 1; i > 0;)
        {
            --i;
            nStride *= anForwardBlockSizes[m_anReverseOrder[i + 1]];
            stack[m_anReverseOrder[i]].dst_inc_offset = nStride;
        }

        stack[nDims - 1].src_inc_offset = nSourceSize;
        nStride = nSourceSize;
        for (size_t i = nDims - 1; i > 0;)
        {
            --i;
            nStride *= anForwardBlockSizes[i + 1];
            stack[i].src_inc_offset = nStride;
        }
    }

    stack[0].src_ptr = abySrc.data();
    stack[0].dst_ptr = &abyDst[0];

    size_t dimIdx = 0;
lbl_next_depth:
    if (dimIdx == nDims)
    {
        void *dst_ptr = stack[nDims].dst_ptr;
        const void *src_ptr = stack[nDims].src_ptr;
        if (nSourceSize == 1)
            *stack[nDims].dst_ptr = *stack[nDims].src_ptr;
        else if (nSourceSize == 2)
            *static_cast<uint16_t *>(dst_ptr) =
                *static_cast<const uint16_t *>(src_ptr);
        else if (nSourceSize == 4)
            *static_cast<uint32_t *>(dst_ptr) =
                *static_cast<const uint32_t *>(src_ptr);
        else if (nSourceSize == 8)
            *static_cast<uint64_t *>(dst_ptr) =
                *static_cast<const uint64_t *>(src_ptr);
        else
            memcpy(dst_ptr, src_ptr, nSourceSize);
    }
    else
    {
        stack[dimIdx].nIters = anForwardBlockSizes[dimIdx];
        while (true)
        {
            dimIdx++;
            stack[dimIdx].src_ptr = stack[dimIdx - 1].src_ptr;
            stack[dimIdx].dst_ptr = stack[dimIdx - 1].dst_ptr;
            goto lbl_next_depth;
        lbl_return_to_caller:
            dimIdx--;
            if ((--stack[dimIdx].nIters) == 0)
                break;
            stack[dimIdx].src_ptr += stack[dimIdx].src_inc_offset;
            stack[dimIdx].dst_ptr += stack[dimIdx].dst_inc_offset;
        }
    }
    if (dimIdx > 0)
        goto lbl_return_to_caller;

    return true;
}

/************************************************************************/
/*                    ZarrV3CodecTranspose::Encode()                    */
/************************************************************************/

bool ZarrV3CodecTranspose::Encode(const ZarrByteVectorQuickResize &abySrc,
                                  ZarrByteVectorQuickResize &abyDst) const
{
    CPLAssert(!IsNoOp());

    return Transpose(abySrc, abyDst, true, m_oInputArrayMetadata.anBlockSizes);
}

/************************************************************************/
/*                    ZarrV3CodecTranspose::Decode()                    */
/************************************************************************/

bool ZarrV3CodecTranspose::Decode(const ZarrByteVectorQuickResize &abySrc,
                                  ZarrByteVectorQuickResize &abyDst) const
{
    CPLAssert(!IsNoOp());

    return Transpose(abySrc, abyDst, false, m_oInputArrayMetadata.anBlockSizes);
}

/************************************************************************/
/*                   ZarrV3CodecTranspose::DecodePartial()              */
/************************************************************************/

bool ZarrV3CodecTranspose::DecodePartial(
    VSIVirtualHandle * /* poFile */, const ZarrByteVectorQuickResize &abySrc,
    ZarrByteVectorQuickResize &abyDst, std::vector<size_t> &anStartIdx,
    std::vector<size_t> &anCount)
{
    CPLAssert(anStartIdx.size() == m_oInputArrayMetadata.anBlockSizes.size());
    CPLAssert(anStartIdx.size() == anCount.size());

    Reorder1DInverse(anStartIdx);
    Reorder1DInverse(anCount);

    // Note that we don't need to take anStartIdx into account for the
    // transpose operation, as abySrc corresponds to anStartIdx.
    return Transpose(abySrc, abyDst, false, anCount);
}

/************************************************************************/
/*                ZarrV3CodecCRC32C::ZarrV3CodecCRC32C()                */
/************************************************************************/

constexpr const char *GDAL_ZARR_CHECK_CRC = "GDAL_ZARR_CHECK_CRC";

ZarrV3CodecCRC32C::ZarrV3CodecCRC32C()
    : ZarrV3Codec(NAME),
      m_bCheckCRC(
          // Undocumented config option. Only used for tests!
          CPLTestBool(CPLGetConfigOption(GDAL_ZARR_CHECK_CRC, "YES")))
{
}

/************************************************************************/
/*                      ZarrV3CodecCRC32C::Clone()                      */
/************************************************************************/

std::unique_ptr<ZarrV3Codec> ZarrV3CodecCRC32C::Clone() const
{
    auto psClone = std::make_unique<ZarrV3CodecCRC32C>();
    ZarrArrayMetadata oOutputArrayMetadata;
    psClone->InitFromConfiguration(m_oConfiguration, m_oInputArrayMetadata,
                                   oOutputArrayMetadata);
    return psClone;
}

/************************************************************************/
/*               ZarrV3CodecCRC32C::InitFromConfiguration()             */
/************************************************************************/

bool ZarrV3CodecCRC32C::InitFromConfiguration(
    const CPLJSONObject &configuration,
    const ZarrArrayMetadata &oInputArrayMetadata,
    ZarrArrayMetadata &oOutputArrayMetadata)
{
    m_oConfiguration = configuration.Clone();
    m_oInputArrayMetadata = oInputArrayMetadata;
    oOutputArrayMetadata = oInputArrayMetadata;

    // GDAL extension for tests !!!
    if (!m_oConfiguration.GetBool("check_crc", true))
        m_bCheckCRC = false;

    return true;
}

/************************************************************************/
/*                           ComputeCRC32C()                            */
/************************************************************************/

static uint32_t ComputeCRC32C(const GByte *pabyIn, size_t nLength)
{
    crc32c_init();
    return crc32c(0, pabyIn, nLength);
}

/************************************************************************/
/*                     ZarrV3CodecCRC32C::Encode()                      */
/************************************************************************/

bool ZarrV3CodecCRC32C::Encode(const ZarrByteVectorQuickResize &abySrc,
                               ZarrByteVectorQuickResize &abyDst) const
{
    abyDst.clear();
    abyDst.insert(abyDst.end(), abySrc.begin(), abySrc.end());

    const uint32_t nComputedCRC_le =
        CPL_LSBWORD32(ComputeCRC32C(abySrc.data(), abySrc.size()));
    const GByte *pabyCRC = reinterpret_cast<const GByte *>(&nComputedCRC_le);
    abyDst.insert(abyDst.end(), pabyCRC, pabyCRC + sizeof(uint32_t));

    return true;
}

/************************************************************************/
/*                        ZarrV3CodecCRC32C::Decode()                   */
/************************************************************************/

bool ZarrV3CodecCRC32C::Decode(const ZarrByteVectorQuickResize &abySrc,
                               ZarrByteVectorQuickResize &abyDst) const
{
    if (abySrc.size() < sizeof(uint32_t))
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "CRC32C decoder: not enough input bytes");
        return false;
    }

    const size_t nSrcLen = abySrc.size() - sizeof(uint32_t);
    abyDst.clear();
    abyDst.insert(abyDst.end(), abySrc.begin(), abySrc.begin() + nSrcLen);

    if (m_bCheckCRC)
    {
        const uint32_t nComputedCRC =
            ComputeCRC32C(abyDst.data(), abyDst.size());
        const uint32_t nExpectedCRC = CPL_LSBUINT32PTR(abySrc.data() + nSrcLen);
        if (nComputedCRC != nExpectedCRC)
        {
            CPLError(
                CE_Failure, CPLE_AppDefined,
                "CRC32C decoder: computed CRC value is %08X whereas expected "
                "value is %08X",
                nComputedCRC, nExpectedCRC);
            return false;
        }
    }

    return true;
}

/************************************************************************/
/*                    ZarrV3CodecSequence::Clone()                      */
/************************************************************************/

std::unique_ptr<ZarrV3CodecSequence> ZarrV3CodecSequence::Clone() const
{
    auto poClone = std::make_unique<ZarrV3CodecSequence>(m_oInputArrayMetadata);
    for (const auto &poCodec : m_apoCodecs)
        poClone->m_apoCodecs.emplace_back(poCodec->Clone());
    poClone->m_oCodecArray = m_oCodecArray.Clone();
    poClone->m_bPartialDecodingPossible = m_bPartialDecodingPossible;
    return poClone;
}

/************************************************************************/
/*                    ZarrV3CodecSequence::InitFromJson()               */
/************************************************************************/

bool ZarrV3CodecSequence::InitFromJson(const CPLJSONObject &oCodecs,
                                       ZarrArrayMetadata &oOutputArrayMetadata)
{
    if (oCodecs.GetType() != CPLJSONObject::Type::Array)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "codecs is not an array");
        return false;
    }
    auto oCodecsArray = oCodecs.ToArray();

    ZarrArrayMetadata oInputArrayMetadata = m_oInputArrayMetadata;
    ZarrV3Codec::IOType eLastType = ZarrV3Codec::IOType::ARRAY;
    std::string osLastCodec;

    const auto InsertImplicitEndianCodecIfNeeded =
        [this, &oInputArrayMetadata, &eLastType, &osLastCodec]()
    {
        CPL_IGNORE_RET_VAL(this);
        if (eLastType == ZarrV3Codec::IOType::ARRAY &&
            oInputArrayMetadata.oElt.nativeSize > 1)
        {
            CPLError(CE_Warning, CPLE_AppDefined,
                     "'bytes' codec missing. Assuming little-endian storage, "
                     "but such tolerance may be removed in future versions");
            auto poEndianCodec = std::make_unique<ZarrV3CodecBytes>();
            ZarrArrayMetadata oTmpOutputArrayMetadata;
            poEndianCodec->InitFromConfiguration(
                ZarrV3CodecBytes::GetConfiguration(true), oInputArrayMetadata,
                oTmpOutputArrayMetadata);
            oInputArrayMetadata = std::move(oTmpOutputArrayMetadata);
            eLastType = poEndianCodec->GetOutputType();
            osLastCodec = poEndianCodec->GetName();
            if constexpr (!CPL_IS_LSB)
            {
                // Insert a little endian codec if we are on a big endian target
                m_apoCodecs.emplace_back(std::move(poEndianCodec));
            }
        }
    };

    bool bShardingFound = false;
    std::vector<size_t> anBlockSizesBeforeSharding;
    for (const auto &oCodec : oCodecsArray)
    {
        if (oCodec.GetType() != CPLJSONObject::Type::Object)
        {
            CPLError(CE_Failure, CPLE_AppDefined, "codecs[] is not an array");
            return false;
        }
        const auto osName = oCodec["name"].ToString();
        std::unique_ptr<ZarrV3Codec> poCodec;
        if (osName == ZarrV3CodecGZip::NAME)
            poCodec = std::make_unique<ZarrV3CodecGZip>();
        else if (osName == ZarrV3CodecBlosc::NAME)
            poCodec = std::make_unique<ZarrV3CodecBlosc>();
        else if (osName == ZarrV3CodecZstd::NAME)
            poCodec = std::make_unique<ZarrV3CodecZstd>();
        else if (osName == ZarrV3CodecBytes::NAME ||
                 osName == "endian" /* endian is the old name */)
            poCodec = std::make_unique<ZarrV3CodecBytes>();
        else if (osName == ZarrV3CodecTranspose::NAME)
            poCodec = std::make_unique<ZarrV3CodecTranspose>();
        else if (osName == ZarrV3CodecCRC32C::NAME)
            poCodec = std::make_unique<ZarrV3CodecCRC32C>();
        else if (osName == ZarrV3CodecShardingIndexed::NAME)
        {
            bShardingFound = true;
            poCodec = std::make_unique<ZarrV3CodecShardingIndexed>();
        }
        else
        {
            CPLError(CE_Failure, CPLE_NotSupported, "Unsupported codec: %s",
                     osName.c_str());
            return false;
        }

        if (poCodec->GetInputType() == ZarrV3Codec::IOType::ARRAY)
        {
            if (eLastType == ZarrV3Codec::IOType::BYTES)
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Cannot chain codec %s with %s",
                         poCodec->GetName().c_str(), osLastCodec.c_str());
                return false;
            }
        }
        else
        {
            InsertImplicitEndianCodecIfNeeded();
        }

        ZarrArrayMetadata oStepOutputArrayMetadata;
        if (osName == ZarrV3CodecShardingIndexed::NAME)
        {
            anBlockSizesBeforeSharding = oInputArrayMetadata.anBlockSizes;
        }
        if (!poCodec->InitFromConfiguration(oCodec["configuration"],
                                            oInputArrayMetadata,
                                            oStepOutputArrayMetadata))
        {
            return false;
        }
        oInputArrayMetadata = std::move(oStepOutputArrayMetadata);
        eLastType = poCodec->GetOutputType();
        osLastCodec = poCodec->GetName();

        if (!poCodec->IsNoOp())
            m_apoCodecs.emplace_back(std::move(poCodec));
    }

    if (bShardingFound)
    {
        m_bPartialDecodingPossible =
            (m_apoCodecs.back()->GetName() == ZarrV3CodecShardingIndexed::NAME);
        if (!m_bPartialDecodingPossible)
        {
            m_bPartialDecodingPossible = false;
            CPLError(
                CE_Warning, CPLE_AppDefined,
                "Sharding codec found, but not in last position. Consequently "
                "partial shard decoding will not be possible");
            oInputArrayMetadata.anBlockSizes = anBlockSizesBeforeSharding;
        }
    }

    InsertImplicitEndianCodecIfNeeded();

    m_oCodecArray = oCodecs.Clone();
    oOutputArrayMetadata = std::move(oInputArrayMetadata);
    return true;
}

/************************************************************************/
/*                  ZarrV3CodecBytes::AllocateBuffer()                 */
/************************************************************************/

bool ZarrV3CodecSequence::AllocateBuffer(ZarrByteVectorQuickResize &abyBuffer,
                                         size_t nEltCount)
{
    if (!m_apoCodecs.empty())
    {
        const size_t nRawSize =
            nEltCount * m_oInputArrayMetadata.oElt.nativeSize;
        // Grow the temporary buffer a bit beyond the uncompressed size
        const size_t nMaxSize = nRawSize + nRawSize / 3 + 64;
        try
        {
            m_abyTmp.resize(nMaxSize);
        }
        catch (const std::exception &e)
        {
            CPLError(CE_Failure, CPLE_OutOfMemory, "%s", e.what());
            return false;
        }
        m_abyTmp.resize(nRawSize);

        // Grow the input/output buffer too if we have several steps
        if (m_apoCodecs.size() >= 2 && abyBuffer.capacity() < nMaxSize)
        {
            const size_t nSize = abyBuffer.size();
            try
            {
                abyBuffer.resize(nMaxSize);
            }
            catch (const std::exception &e)
            {
                CPLError(CE_Failure, CPLE_OutOfMemory, "%s", e.what());
                return false;
            }
            abyBuffer.resize(nSize);
        }
    }
    return true;
}

/************************************************************************/
/*                    ZarrV3CodecSequence::Encode()                     */
/************************************************************************/

bool ZarrV3CodecSequence::Encode(ZarrByteVectorQuickResize &abyBuffer)
{
    if (!AllocateBuffer(abyBuffer,
                        MultiplyElements(m_oInputArrayMetadata.anBlockSizes)))
        return false;
    for (const auto &poCodec : m_apoCodecs)
    {
        if (!poCodec->Encode(abyBuffer, m_abyTmp))
            return false;
        std::swap(abyBuffer, m_abyTmp);
    }
    return true;
}

/************************************************************************/
/*                    ZarrV3CodecSequence::Decode()                     */
/************************************************************************/

bool ZarrV3CodecSequence::Decode(ZarrByteVectorQuickResize &abyBuffer)
{
    if (!AllocateBuffer(abyBuffer,
                        MultiplyElements(m_oInputArrayMetadata.anBlockSizes)))
        return false;
    for (auto iter = m_apoCodecs.rbegin(); iter != m_apoCodecs.rend(); ++iter)
    {
        const auto &poCodec = *iter;
        if (!poCodec->Decode(abyBuffer, m_abyTmp))
            return false;
        std::swap(abyBuffer, m_abyTmp);
    }
    return true;
}

/************************************************************************/
/*                ZarrV3CodecSequence::DecodePartial()                  */
/************************************************************************/

bool ZarrV3CodecSequence::DecodePartial(VSIVirtualHandle *poFile,
                                        ZarrByteVectorQuickResize &abyBuffer,
                                        const std::vector<size_t> &anStartIdxIn,
                                        const std::vector<size_t> &anCountIn)
{
    CPLAssert(anStartIdxIn.size() == m_oInputArrayMetadata.anBlockSizes.size());
    CPLAssert(anStartIdxIn.size() == anCountIn.size());

    if (!AllocateBuffer(abyBuffer, MultiplyElements(anCountIn)))
        return false;

    // anStartIdxIn and anCountIn are expressed in the shape *before* encoding
    // We need to apply the potential transpositions before submitting them
    // to the decoder of the Array->Bytes decoder
    std::vector<size_t> anStartIdx(anStartIdxIn);
    std::vector<size_t> anCount(anCountIn);
    for (auto &poCodec : m_apoCodecs)
    {
        poCodec->ChangeArrayShapeForward(anStartIdx, anCount);
    }

    for (auto iter = m_apoCodecs.rbegin(); iter != m_apoCodecs.rend(); ++iter)
    {
        const auto &poCodec = *iter;

        if (!poCodec->DecodePartial(poFile, abyBuffer, m_abyTmp, anStartIdx,
                                    anCount))
            return false;
        std::swap(abyBuffer, m_abyTmp);
    }
    return true;
}

/************************************************************************/
/*           ZarrV3CodecSequence::GetInnerMostBlockSize()               */
/************************************************************************/

std::vector<size_t> ZarrV3CodecSequence::GetInnerMostBlockSize(
    const std::vector<size_t> &anOuterBlockSize) const
{
    auto chunkSize = anOuterBlockSize;
    for (auto iter = m_apoCodecs.rbegin(); iter != m_apoCodecs.rend(); ++iter)
    {
        const auto &poCodec = *iter;
        if (m_bPartialDecodingPossible ||
            poCodec->GetName() != ZarrV3CodecShardingIndexed::NAME)
        {
            chunkSize = poCodec->GetInnerMostBlockSize(chunkSize);
        }
    }
    return chunkSize;
}

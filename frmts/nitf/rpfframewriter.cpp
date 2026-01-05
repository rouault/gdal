/******************************************************************************
 *
 * Project:  NITF Read/Write Library
 * Purpose:  Creates RPFHDR, RPFIMG, RPFDES TREs and RPF image data
 * Author:   Even Rouault, even dot rouault at spatialys dot com
 *
 **********************************************************************
 * Copyright (c) 2026, T-Kartor
 *
 * SPDX-License-Identifier: MIT
 ****************************************************************************/

#include "rpfframewriter.h"

#include "cpl_string.h"
#include "gdal_colortable.h"
#include "gdal_dataset.h"
#include "gdal_geotransform.h"
#include "gdal_rasterband.h"
#include "offsetpatcher.h"
#include "nitflib.h"
#include "kdtree_vq_cadrg.h"

#include <algorithm>
#include <array>
#include <map>
#include <vector>

// Refer to following documents to (hopefully) understand that file:
// MIL-C-89038, CADRG:          http://everyspec.com/MIL-PRF/MIL-PRF-080000-99999/MIL-PRF-89038_25371/
// MIL-STD-2411, RPF:           http://everyspec.com/MIL-STD/MIL-STD-2000-2999/MIL-STD-2411_6903/
// MIL-STD-2411-1, RPF details: http://everyspec.com/MIL-STD/MIL-STD-2000-2999/MIL-STD-2411-1_6909/
// MIL-STD-2411-2, RPF-in-NITF: http://everyspec.com/MIL-STD/MIL-STD-2000-2999/MIL-STD-2411-2_6908/
// MIL-A-89007, ADRG:           http://everyspec.com/MIL-SPECS/MIL-SPECS-MIL-A/MIL-A-89007_51725/
// MIL-STD-188/199, VQ NITF:    http://everyspec.com/MIL-STD/MIL-STD-0100-0299/MIL_STD_188_199_1730/

constexpr int SUBSAMPLING = 4;
constexpr int BLOCK_SIZE = 256;
constexpr int CODEBOOK_MAX_SIZE = 4096;

/************************************************************************/
/*                           StrPadTruncate()                           */
/************************************************************************/

static std::string StrPadTruncate(const std::string &osIn, size_t nSize)
{
    std::string osOut(osIn);
    osOut.resize(nSize, ' ');
    return osOut;
}

/************************************************************************/
/*                        Create_CADRG_RPFHDR()                         */
/************************************************************************/

static void Create_CADRG_RPFHDR(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                                const std::string &osFilename,
                                CPLStringList &aosOptions)
{
    auto poRPFHDR = offsetPatcher->CreateBuffer(
        "RPFHDR", /* bEndiannessIsLittle = */ false);
    CPLAssert(poRPFHDR);
    poRPFHDR->AppendByte(0);  // big endian order
    poRPFHDR->AppendUInt16RefForSizeOfBuffer("RPFHDR");
    poRPFHDR->AppendString(
        StrPadTruncate(CPLGetFilename(osFilename.c_str()), 12));
    poRPFHDR->AppendByte(1);  // update indicator: full replacement
    poRPFHDR->AppendString("MIL-C-89038    ");  // GOVERNING_STANDARD_NUMBER
    poRPFHDR->AppendString("19941006");         // GOVERNING_STANDARD_DATE
    poRPFHDR->AppendString("U");   // SECURITY_CLASSIFICATION: unclassified
    poRPFHDR->AppendString("  ");  // SECURITY_COUNTRY_INTERNATIONAL_CODE
    poRPFHDR->AppendString("  ");  // SECURITY_RELEASE_MARKING
    poRPFHDR->AppendUInt32RefForOffset("LOCATION_COMPONENT_LOCATION");

    char *pszEscaped = CPLEscapeString(
        reinterpret_cast<const char *>(poRPFHDR->GetBuffer().data()),
        static_cast<int>(poRPFHDR->GetBuffer().size()),
        CPLES_BackslashQuotable);
    aosOptions.AddString(
        std::string("FILE_TRE=RPFHDR=").append(pszEscaped).c_str());
    CPLFree(pszEscaped);
}

/************************************************************************/
/*                   Create_CADRG_LocationComponent()                   */
/************************************************************************/

static void
Create_CADRG_LocationComponent(GDALOffsetPatcher::OffsetPatcher *offsetPatcher)
{
    auto poBuffer = offsetPatcher->CreateBuffer(
        "LocationComponent", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareOffsetAtCurrentPosition("LOCATION_COMPONENT_LOCATION");

    static const struct
    {
        uint16_t locationId;
        const char *locationBufferName;
        const char *locationOffsetName;
    } asLocations[] = {
        {LID_CompressionLookupSubsection /* 132 */,
         "CompressionLookupSubsection", "COMPRESSION_LOOKUP_LOCATION"},
        {LID_ColormapSubsection /* 135 */, "ColormapSubsection",
         "COLORMAP_LOCATION"},
        {LID_SpatialDataSubsection /* 140 */, "SpatialDataSubsection",
         "SPATIAL_DATA_SUBSECTION_LOCATION"},
    };

    std::string sumOfSizes;
    uint16_t nComponents = 0;
    for (const auto &sLocation : asLocations)
    {
        ++nComponents;
        if (!sumOfSizes.empty())
            sumOfSizes += '+';
        sumOfSizes += sLocation.locationBufferName;
    }

    constexpr uint16_t COMPONENT_LOCATION_OFFSET = 14;
    constexpr uint16_t COMPONENT_LOCATION_RECORD_LENGTH = 10;
    poBuffer->AppendUInt16RefForSizeOfBuffer("LocationComponent");
    poBuffer->AppendUInt32(COMPONENT_LOCATION_OFFSET);
    poBuffer->AppendUInt16(nComponents);
    poBuffer->AppendUInt16(COMPONENT_LOCATION_RECORD_LENGTH);
    // COMPONENT_AGGREGATE_LENGTH
    poBuffer->AppendUInt32RefForSizeOfBuffer(sumOfSizes);

    for (const auto &sLocation : asLocations)
    {
        poBuffer->AppendUInt16(sLocation.locationId);
        poBuffer->AppendUInt32RefForSizeOfBuffer(sLocation.locationBufferName);
        poBuffer->AppendUInt32RefForOffset(sLocation.locationOffsetName);
    }
}

/************************************************************************/
/*                    Create_CADRG_ColormapSection()                    */
/************************************************************************/

static void
Create_CADRG_ColormapSection(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                             GDALDataset *poSrcDS)
{
    auto poBuffer = offsetPatcher->CreateBuffer(
        "ColormapSubsection", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareOffsetAtCurrentPosition("COLORMAP_LOCATION");
    poBuffer->AppendUInt32(4);  // COLORMAP_OFFSET_TABLE_OFFSET
    constexpr uint16_t RECORD_LENGTH = 17;
    poBuffer->AppendUInt16(
        RECORD_LENGTH);  // COLOR_GRAYSCALE_OFFSET_RECORD_LENGTH
    const int HEADER_LENGTH = static_cast<int>(poBuffer->GetBuffer().size());

    poBuffer->AppendUInt16(2);  // color/grayscale table id
    poBuffer->AppendUInt32(CADRG_MAX_COLOR_ENTRY_COUNT);  // number of colors
    // 4=R,G,B,M
    constexpr GByte COLOR_TABLE_ENTRY_SIZE = 4;
    poBuffer->AppendByte(
        COLOR_TABLE_ENTRY_SIZE);  // color/grayscale element length
    poBuffer->AppendUInt16(0);    // histogram record length (omitted)
    poBuffer->AppendUInt32(HEADER_LENGTH + RECORD_LENGTH);
    // color/grayscale table offset (omitted)
    poBuffer->AppendUInt32(UINT32_MAX);

    // Write color table
    const auto poCT = poSrcDS->GetRasterBand(1)->GetColorTable();
    for (int i = 0; i < CADRG_MAX_COLOR_ENTRY_COUNT; ++i)
    {
        if (i < poCT->GetColorEntryCount())
        {
            auto psEntry = poCT->GetColorEntry(i);
            poBuffer->AppendByte(static_cast<GByte>(psEntry->c1));
            poBuffer->AppendByte(static_cast<GByte>(psEntry->c2));
            poBuffer->AppendByte(static_cast<GByte>(psEntry->c3));
            // Standard formula to convert R,G,B to gray scale level
            const int M =
                (psEntry->c1 * 299 + psEntry->c2 * 587 + psEntry->c3 * 114) /
                1000;
            poBuffer->AppendByte(static_cast<GByte>(M));
        }
        else
        {
            poBuffer->AppendUInt32(0);
        }
    }
}

/************************************************************************/
/*                      RPFFrameCreateCADRG_TREs()                      */
/************************************************************************/

bool RPFFrameCreateCADRG_TREs(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                              const std::string &osFilename,
                              GDALDataset *poSrcDS, CPLStringList &aosOptions)
{
    Create_CADRG_RPFHDR(offsetPatcher, osFilename, aosOptions);

    // Create buffers that will be written into file by RPFFrameWriteCADRG_RPFIMG()s
    Create_CADRG_LocationComponent(offsetPatcher);
    Create_CADRG_ColormapSection(offsetPatcher, poSrcDS);
    return true;
}

/************************************************************************/
/*                     RPFFrameWriteCADRG_RPFIMG()                      */
/************************************************************************/

bool RPFFrameWriteCADRG_RPFIMG(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                               VSILFILE *fp, int &nUDIDL)
{
    std::vector<GDALOffsetPatcher::OffsetPatcherBuffer *> apoBuffers;
    int nContentLength = 0;
    for (const char *pszName : {"LocationComponent", "ColormapSubsection"})
    {
        const auto poBuffer = offsetPatcher->GetBufferFromName(pszName);
        CPLAssert(poBuffer);
        apoBuffers.push_back(poBuffer);
        nContentLength += static_cast<int>(poBuffer->GetBuffer().size());
    }

    CPLAssert(nContentLength <= 99999);
    constexpr const char *pszUDOFL = "000";
    const char *pszTREPrefix = CPLSPrintf("RPFIMG%05d", nContentLength);
    nUDIDL = static_cast<int>(strlen(pszUDOFL) + strlen(pszTREPrefix) +
                              nContentLength);

    // UDIDL
    bool bOK = fp->Write(CPLSPrintf("%05d", nUDIDL), 1, 5) == 5;

    // UDOFL
    bOK &= fp->Write(pszUDOFL, 1, strlen(pszUDOFL)) == strlen(pszUDOFL);

    // UDID
    bOK &= fp->Write(pszTREPrefix, 1, strlen(pszTREPrefix)) ==
           strlen(pszTREPrefix);

    for (auto *poBuffer : apoBuffers)
    {
        poBuffer->DeclareBufferWrittenAtPosition(VSIFTellL(fp));
        bOK &= VSIFWriteL(poBuffer->GetBuffer().data(), 1,
                          poBuffer->GetBuffer().size(),
                          fp) == poBuffer->GetBuffer().size();
    }

    return bOK;
}

/************************************************************************/
/*                    Perform_CADRG_VQ_Compression()                    */
/************************************************************************/

static bool Perform_CADRG_VQ_Compression(
    GDALDataset *poSrcDS, const ColorTableBased4x4Pixels &ctxt,
    std::vector<BucketItem<ColorTableBased4x4Pixels>> &codebook,
    std::vector<short> &VQImage)
{
    const int nY = poSrcDS->GetRasterYSize();
    const int nX = poSrcDS->GetRasterXSize();
    CPLAssert((nY % SUBSAMPLING) == 0);
    CPLAssert((nX % SUBSAMPLING) == 0);

    auto poBand = poSrcDS->GetRasterBand(1);

    std::vector<GByte> pixels;
    if (poBand->ReadRaster(pixels) != CE_None)
        return false;

    const auto poCT = poBand->GetColorTable();
    const int nColorCount =
        std::min(CADRG_MAX_COLOR_ENTRY_COUNT, poCT->GetColorEntryCount());

    struct Occurences
    {
        // number of 4x4 pixel blocks using those 4x4 pixel values
        int nCount = 0;
        // Point to indices in the output image that use that 4x4 pixel blocks
        std::vector<int> anIndicesToOutputImage{};
    };

    // Collect all the occurences of 4x4 pixel values into a map indexed by them
    std::map<Vector<ColorTableBased4x4Pixels>, Occurences> vectorMap;
    for (int j = 0, nOutputIdx = 0; j < nY / SUBSAMPLING; ++j)
    {
        for (int i = 0; i < nX / SUBSAMPLING; ++i, ++nOutputIdx)
        {
            std::array<GByte, SUBSAMPLING * SUBSAMPLING> vals;
            for (int y = 0; y < SUBSAMPLING; ++y)
            {
                for (int x = 0; x < SUBSAMPLING; ++x)
                {
                    const GByte val = pixels[(j * SUBSAMPLING + y) * nX +
                                             (i * SUBSAMPLING + x)];
                    if (val >= nColorCount)
                    {
                        CPLError(CE_Failure, CPLE_AppDefined,
                                 "Out of range pixel value found: %d", val);
                        return false;
                    }
                    vals[SUBSAMPLING * y + x] = val;
                }
            }
            auto &elt = vectorMap[Vector<ColorTableBased4x4Pixels>(vals)];
            ++elt.nCount;
            elt.anIndicesToOutputImage.push_back(nOutputIdx);
        }
    }

    // Convert that map into a std::vector
    std::vector<BucketItem<ColorTableBased4x4Pixels>> vectors;
    vectors.reserve(vectorMap.size());
    for (auto &[key, value] : vectorMap)
    {
        vectors.emplace_back(key, value.nCount,
                             std::move(value.anIndicesToOutputImage));
    }
    vectorMap.clear();

    // Create the KD-Tree
    PNNKDTree<ColorTableBased4x4Pixels> kdtree;

    // Insert the initial items
    int nCodeCount = kdtree.insert(std::move(vectors), ctxt);
    if (nCodeCount == 0)
        return false;

    // Reduce to the maximum target
    if (nCodeCount > CODEBOOK_MAX_SIZE)
    {
        const int nNewCodeCount =
            kdtree.cluster(nCodeCount, CODEBOOK_MAX_SIZE, ctxt);
        if (nNewCodeCount == 0)
            return false;
        CPLDebug("NITF", "VQ compression: reducing from %d codes to %d",
                 nCodeCount, nNewCodeCount);
        nCodeCount = nNewCodeCount;
    }
    else
    {
        CPLDebug("NITF",
                 "Already less than %d codes. VQ compression is lossless",
                 CODEBOOK_MAX_SIZE);
    }

    // Create the code book and the target VQ-compressed image.
    codebook.reserve(nCodeCount);
    VQImage.resize((nY / SUBSAMPLING) * (nX / SUBSAMPLING));
    kdtree.iterateOverLeaves(
        [&codebook, &VQImage](PNNKDTree<ColorTableBased4x4Pixels> &node)
        {
            for (auto &item : node.bucketItems())
            {
                const int i = static_cast<int>(codebook.size());
                for (const auto idx : item.m_origVectorIndices)
                {
                    VQImage[idx] = static_cast<short>(i);
                }
                codebook.push_back(std::move(item));
            }
        });

    return true;
}

/************************************************************************/
/*              Write_CADRG_CompressionLookupSubSection()               */
/************************************************************************/

static bool Write_CADRG_CompressionLookupSubSection(
    GDALOffsetPatcher::OffsetPatcher *offsetPatcher, VSILFILE *fp,
    const std::vector<BucketItem<ColorTableBased4x4Pixels>> &codebook)
{
    auto poBuffer = offsetPatcher->CreateBuffer(
        "CompressionLookupSubsection", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareBufferWrittenAtPosition(fp->Tell());
    poBuffer->DeclareOffsetAtCurrentPosition("COMPRESSION_LOOKUP_LOCATION");
    poBuffer->AppendUInt32(6);  // COMPRESSION_LOOKUP_OFFSET_TABLE_OFFSET
    // COMPRESSION_LOOKUP_TABLE_OFFSET_RECORD_LENGTH
    poBuffer->AppendUInt16(14);

    constexpr int OFFSET_OF_FIRST_LOOKUP_TABLE = 62;
    constexpr uint16_t NUMBER_OF_VALUES_PER_COMPRESSION_RECORDS =
        static_cast<uint16_t>(SUBSAMPLING);
    constexpr int NUMBER_OF_COMPRESSION_LOOKUP_OFFSET_RECORDS = 4;
    for (int i = 0; i < NUMBER_OF_COMPRESSION_LOOKUP_OFFSET_RECORDS; ++i)
    {
        // COMPRESSION_LOOKUP_TABLE_ID
        poBuffer->AppendUInt16(static_cast<uint16_t>(i + 1));
        poBuffer->AppendUInt32(
            CODEBOOK_MAX_SIZE);  // NUMBER_OF_COMPRESSION_LOOKUP_RECORDS
        poBuffer->AppendUInt16(NUMBER_OF_VALUES_PER_COMPRESSION_RECORDS);
        poBuffer->AppendUInt16(8);  // COMPRESSION_RECORD_VALUE_BIT_LENGTH
        poBuffer->AppendUInt32(OFFSET_OF_FIRST_LOOKUP_TABLE +
                               CODEBOOK_MAX_SIZE *
                                   NUMBER_OF_VALUES_PER_COMPRESSION_RECORDS *
                                   i);  // COMPRESSION_LOOKUP_TABLE_OFFSET
    }

    for (int row = 0; row < SUBSAMPLING; ++row)
    {
        int i = 0;
        for (; i < static_cast<int>(codebook.size()); ++i)
        {
            for (int j = 0; j < SUBSAMPLING; ++j)
            {
                poBuffer->AppendByte(
                    codebook[i].m_vec.val(row * SUBSAMPLING + j));
            }
        }
        for (; i < CODEBOOK_MAX_SIZE; ++i)
        {
            poBuffer->AppendUInt32(0);
        }
    }

    return fp->Write(poBuffer->GetBuffer().data(), 1,
                     poBuffer->GetBuffer().size()) ==
           poBuffer->GetBuffer().size();
}

/************************************************************************/
/*                 Write_CADRG_SpatialDataSubsection()                  */
/************************************************************************/

static bool Write_CADRG_SpatialDataSubsection(
    GDALOffsetPatcher::OffsetPatcher *offsetPatcher, VSILFILE *fp,
    GDALDataset *poSrcDS, const std::vector<short> &VQImage)
{
    auto poBuffer = offsetPatcher->CreateBuffer(
        "SpatialDataSubsection", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareBufferWrittenAtPosition(fp->Tell());
    poBuffer->DeclareOffsetAtCurrentPosition(
        "SPATIAL_DATA_SUBSECTION_LOCATION");

    const int nWidth = poSrcDS->GetRasterXSize();
    const int nHeight = poSrcDS->GetRasterYSize();
    CPLAssert((nWidth % BLOCK_SIZE) == 0);
    CPLAssert((nHeight % BLOCK_SIZE) == 0);
    const int nSubFramesPerRow = nWidth / BLOCK_SIZE;
    const int nSubFramesPerCol = nHeight / BLOCK_SIZE;
    static_assert((BLOCK_SIZE % SUBSAMPLING) == 0);
    constexpr int SUBFRAME_YSIZE = BLOCK_SIZE / SUBSAMPLING;
    constexpr int SUBFRAME_XSIZE = BLOCK_SIZE / SUBSAMPLING;
    const int nPixelsPerRow = nSubFramesPerRow * SUBFRAME_XSIZE;
    for (int yBlock = 0; yBlock < nSubFramesPerCol; ++yBlock)
    {
        int nOffsetBlock = yBlock * SUBFRAME_YSIZE * nPixelsPerRow;
        for (int xBlock = 0; xBlock < nSubFramesPerRow;
             ++xBlock, nOffsetBlock += SUBFRAME_XSIZE)
        {
            for (int ySubBlock = 0; ySubBlock < SUBFRAME_YSIZE; ySubBlock++)
            {
                int nOffset = nOffsetBlock + ySubBlock * nPixelsPerRow;
                // Combine 2 codes of 12 bits each into 3 bytes
                // This is the reverse of function NITFUncompressVQTile()
                for (int xSubBlock = 0; xSubBlock < SUBFRAME_XSIZE;
                     xSubBlock += 2, nOffset += 2)
                {
                    const int v1 = VQImage[nOffset + 0];
                    const int v2 = VQImage[nOffset + 1];
                    poBuffer->AppendByte(static_cast<GByte>(v1 >> 4));
                    poBuffer->AppendByte(
                        static_cast<GByte>(((v1 & 0xF) << 4) | (v2 >> 8)));
                    poBuffer->AppendByte(static_cast<GByte>(v2 & 0xFF));
                }
            }
        }
    }

    return fp->Write(poBuffer->GetBuffer().data(), 1,
                     poBuffer->GetBuffer().size()) ==
           poBuffer->GetBuffer().size();
}

/************************************************************************/
/*                  RPFFrameCreateCADRG_ImageContent()                  */
/************************************************************************/

bool RPFFrameCreateCADRG_ImageContent(
    GDALOffsetPatcher::OffsetPatcher *offsetPatcher, VSILFILE *fp,
    GDALDataset *poSrcDS)
{
    auto poBand = poSrcDS->GetRasterBand(1);
    const auto poCT = poBand->GetColorTable();
    CPLAssert(poCT);
    std::vector<GByte> vR, vG, vB;
    const int nColorCount =
        std::min(CADRG_MAX_COLOR_ENTRY_COUNT, poCT->GetColorEntryCount());
    for (int i = 0; i < nColorCount; ++i)
    {
        const auto entry = poCT->GetColorEntry(i);
        vR.push_back(static_cast<GByte>(entry->c1));
        vG.push_back(static_cast<GByte>(entry->c2));
        vB.push_back(static_cast<GByte>(entry->c3));
    }
    ColorTableBased4x4Pixels ctxt(vR, vG, vB);

    std::vector<BucketItem<ColorTableBased4x4Pixels>> codebook;
    std::vector<short> VQImage;
    return Perform_CADRG_VQ_Compression(poSrcDS, ctxt, codebook, VQImage) &&
           fp->Seek(0, SEEK_END) == 0 &&
           Write_CADRG_CompressionLookupSubSection(offsetPatcher, fp,
                                                   codebook) &&
           Write_CADRG_SpatialDataSubsection(offsetPatcher, fp, poSrcDS,
                                             VQImage);
}

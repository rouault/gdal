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
/*                            StrPadTruncate()                          */
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
#ifdef INCLUDE_HEADER_AND_LOCATION
    poRPFHDR->DeclareOffsetAtCurrentPosition("HEADER_COMPONENT_LOCATION");
#endif
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
/*                     Create_CADRG_LocationComponent()                 */
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
#ifdef INCLUDE_HEADER_AND_LOCATION
        // While it shouldn't hurt, it doesn't seem idiomatical to include
        // those locations in the location table.
        {LID_HeaderComponent /* 128 */, "RPFHDR", "HEADER_COMPONENT_LOCATION"},
        {LID_LocationComponent /* 129 */, "LocationComponent",
         "LOCATION_COMPONENT_LOCATION"},
#endif
        {LID_CoverageSectionSubheader /* 130 */, "CoverageSectionSubheader",
         "COVERAGE_SECTION_LOCATION"},
        {LID_CompressionSectionSubsection /* 131 */, "CompressionSection",
         "COMPRESSION_SECTION_LOCATION"},
        {LID_CompressionLookupSubsection /* 132 */,
         "CompressionLookupSubsection", "COMPRESSION_LOOKUP_LOCATION"},
        /* no LID_CompressionParameterSubsection = 133 in CADRG */
        {LID_ColorGrayscaleSectionSubheader /* 134 */,
         "ColorGrayscaleSectionSubheader", "COLOR_GRAYSCALE_LOCATION"},
        {LID_ColormapSubsection /* 135 */, "ColormapSubsection",
         "COLORMAP_LOCATION"},
        {LID_ImageDescriptionSubheader /* 136 */, "ImageDescriptionSubheader",
         "IMAGE_DESCRIPTION_SECTION_LOCATION"},
        {LID_ImageDisplayParametersSubheader /* 137 */,
         "ImageDisplayParametersSubheader",
         "IMAGE_DISPLAY_PARAMETERS_SECTION_LOCATION"},
        {LID_MaskSubsection /* 138 */, "MaskSubsection",
         "MASK_SUBSECTION_LOCATION"},
        {LID_SpatialDataSubsection /* 140 */, "SpatialDataSubsection",
         "SPATIAL_DATA_SUBSECTION_LOCATION"},
        {LID_AttributeSectionSubheader /* 141 */, "AttributeSectionSubheader",
         "ATTRIBUTE_SECTION_SUBHEADER_LOCATION"},
        {LID_AttributeSubsection /* 142 */, "AttributeSubsection",
         "ATTRIBUTE_SUBSECTION_LOCATION"},
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
/*                      Create_CADRG_CoverageSection()                  */
/************************************************************************/

constexpr double ARC_B = 400384;

// Content of MIL-A-89007 (ADRG specification), appendix 70, table III
static constexpr struct
{
    int nZone;  // zone number (for northern hemisphere. Add 9 for southern hemisphre)
    int minLat;     // minimum latitude of the zone
    int maxLat;     // maximum latitude of the zone
    double A;       // longitudinal pixel spacing constant at 1:1M
    double B;       // latitudinal pixel spacing constant at 1:1M
    double latRes;  // in microns
    double lonRes;  // in microns
} asARCZoneDefinitions[] = {
    {1, 0, 32, 369664, ARC_B, 99.9, 99.9},
    {2, 32, 48, 302592, ARC_B, 99.9, 99.9},
    {3, 48, 56, 245760, ARC_B, 100.0, 99.9},
    {4, 56, 64, 199168, ARC_B, 99.9, 99.9},
    {5, 64, 68, 163328, ARC_B, 99.7, 99.9},
    {6, 68, 72, 137216, ARC_B, 99.7, 99.9},
    {7, 72, 76, 110080, ARC_B, 99.8, 99.9},
    {8, 76, 80, 84432, ARC_B, 100.0, 99.9},
    {9, 80, 90, ARC_B, ARC_B, 99.9, 99.9},
};

static int GetARCZoneFromLat(double dfLat)
{
    for (const auto &sZoneDef : asARCZoneDefinitions)
    {
        if (std::fabs(dfLat) >= sZoneDef.minLat &&
            std::fabs(dfLat) <= sZoneDef.maxLat)
        {
            return dfLat >= 0 ? sZoneDef.nZone : sZoneDef.nZone + 9;
        }
    }
    return 0;
}

/************************************************************************/
/*                      Create_CADRG_CoverageSection()                  */
/************************************************************************/

static bool
Create_CADRG_CoverageSection(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                             GDALDataset *poSrcDS)
{
    auto poBuffer = offsetPatcher->CreateBuffer(
        "CoverageSectionSubheader", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareOffsetAtCurrentPosition("COVERAGE_SECTION_LOCATION");

    GDALGeoTransform gt;
    if (poSrcDS->GetGeoTransform(gt) != CE_None)
        return false;

    // Upper left corner lat, lon
    poBuffer->AppendFloat64(gt[3]);
    poBuffer->AppendFloat64(gt[0]);
    // Lower left corner lat, lon
    poBuffer->AppendFloat64(gt[3] + gt[5] * poSrcDS->GetRasterYSize());
    poBuffer->AppendFloat64(gt[0]);
    // Upper right corner lat, lon
    poBuffer->AppendFloat64(gt[3]);
    poBuffer->AppendFloat64(gt[0] + gt[1] * poSrcDS->GetRasterXSize());
    // Lower right corner lat, lon
    poBuffer->AppendFloat64(gt[3] + gt[5] * poSrcDS->GetRasterYSize());
    poBuffer->AppendFloat64(gt[0] + gt[1] * poSrcDS->GetRasterXSize());

    const double dfMeanLat = gt[3] + gt[5] / 2 * poSrcDS->GetRasterYSize();
    const int nZone = GetARCZoneFromLat(dfMeanLat);
    if (nZone == 0)
        return false;
    const int nZoneIdx = (nZone - 1) % 9;
    const auto &sZoneDef = asARCZoneDefinitions[nZoneIdx];

    const double REF_SCALE = 1e6;
    // Cf MIL-A-89007 (ADRG specification), appendix 70, table III
    const double SCALE = 1e6;  // FIXME
    const double N = REF_SCALE / SCALE;

    constexpr double RATIO_DPI_CADRG_OVER_ADRG = 150.0 / 100.0;

    // Cf MIL-C-89038 (CADRG specification), para 60.1.1 and following
    const double B_s = sZoneDef.B * N;
    constexpr int ADRG_BLOCK_SIZE = 512;
    const double latCst_ADRG =
        std::ceil(B_s / ADRG_BLOCK_SIZE) * ADRG_BLOCK_SIZE;
    const double latCst_CADRG =
        std::round(latCst_ADRG / RATIO_DPI_CADRG_OVER_ADRG / 4 / BLOCK_SIZE) *
        BLOCK_SIZE;
    const double latInterval = 90.0 / latCst_CADRG;
    const double latResolution =
        sZoneDef.latRes / N * latCst_ADRG / (4 * latCst_CADRG);

    const double A_s = sZoneDef.A * N;
    const double lonCst_ADRG =
        std::ceil(A_s / ADRG_BLOCK_SIZE) * ADRG_BLOCK_SIZE;
    const double lonCst_CADRG =
        std::round(lonCst_ADRG / RATIO_DPI_CADRG_OVER_ADRG / BLOCK_SIZE) *
        BLOCK_SIZE;
    const double lonInterval = 360.0 / latCst_CADRG;
    const double lonResolution =
        sZoneDef.lonRes / N * lonCst_ADRG / lonCst_CADRG;

    poBuffer->AppendFloat64(latResolution);
    poBuffer->AppendFloat64(lonResolution);
    poBuffer->AppendFloat64(latInterval);
    poBuffer->AppendFloat64(lonInterval);
    return true;
}

/************************************************************************/
/*                    Create_CADRG_ColorGrayscaleSection()              */
/************************************************************************/

static void Create_CADRG_ColorGrayscaleSection(
    GDALOffsetPatcher::OffsetPatcher *offsetPatcher)
{
    auto poBuffer = offsetPatcher->CreateBuffer(
        "ColorGrayscaleSectionSubheader", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareOffsetAtCurrentPosition("COLOR_GRAYSCALE_LOCATION");
    poBuffer->AppendByte(1);  // NUMBER_OF_COLOR_GRAYSCALE_OFFSET_RECORDS
    poBuffer->AppendByte(0);  // NUMBER_OF_COLOR_CONVERTER_OFFSET_RECORDS
    // EXTERNAL_COLOR_GRAYSCALE_FILENAME
    poBuffer->AppendString(std::string(12, ' '));
}

/************************************************************************/
/*                    Create_CADRG_ImageDescriptionSection()            */
/************************************************************************/

static void Create_CADRG_ImageDescriptionSection(
    GDALOffsetPatcher::OffsetPatcher *offsetPatcher, GDALDataset *poSrcDS)
{
    CPLAssert((poSrcDS->GetRasterXSize() % BLOCK_SIZE) == 0);
    CPLAssert((poSrcDS->GetRasterYSize() % BLOCK_SIZE) == 0);
    CPLAssert(poSrcDS->GetRasterXSize() <= UINT16_MAX * BLOCK_SIZE);
    CPLAssert(poSrcDS->GetRasterYSize() <= UINT16_MAX * BLOCK_SIZE);
    const uint16_t nSubFramesPerRow =
        static_cast<uint16_t>(poSrcDS->GetRasterXSize() / BLOCK_SIZE);
    const uint16_t nSubFramesPerCol =
        static_cast<uint16_t>(poSrcDS->GetRasterYSize() / BLOCK_SIZE);
    CPLAssert(nSubFramesPerRow * nSubFramesPerCol < UINT16_MAX);

    auto poBuffer = offsetPatcher->CreateBuffer(
        "ImageDescriptionSubheader", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareOffsetAtCurrentPosition(
        "IMAGE_DESCRIPTION_SECTION_LOCATION");
    poBuffer->AppendUInt16(1);  // NUMBER_OF_SPECTRAL_GROUPS
    poBuffer->AppendUInt16(static_cast<uint16_t>(
        nSubFramesPerRow * nSubFramesPerCol));  // NUMBER_OF_SUBFRAME_TABLES
    poBuffer->AppendUInt16(1);  // NUMBER_OF_SPECTRAL_BAND_TABLES
    poBuffer->AppendUInt16(1);  // NUMBER_OF_SPECTRAL_BAND_LINES_PER_IMAGE_ROW
    poBuffer->AppendUInt16(
        nSubFramesPerRow);  // NUMBER_OF_SUBFRAME_IN_EAST_WEST_DIRECTION
    poBuffer->AppendUInt16(
        nSubFramesPerCol);  // NUMBER_OF_SUBFRAME_IN_NORTH_SOUTH_DIRECTION
    poBuffer->AppendUInt32(
        BLOCK_SIZE);  // NUMBER_OF_OUTPUT_COLUMNS_PER_SUBFRAME
    poBuffer->AppendUInt32(BLOCK_SIZE);  // NUMBER_OF_OUTPUT_ROWS_PER_SUBFRAME
    poBuffer->AppendUInt32(UINT32_MAX);  // SUBFRAME_MASK_TABLE_OFFSET
    poBuffer->AppendUInt32(UINT32_MAX);  // TRANSPARENCY_MASK_TABLE_OFFSET
}

/************************************************************************/
/*                      Create_CADRG_ColormapSection()                  */
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
    const GByte COLOR_TABLE_ENTRY_SIZE = 4;
    poBuffer->AppendByte(
        COLOR_TABLE_ENTRY_SIZE);  // color/grayscale element length
    poBuffer->AppendUInt16(
        static_cast<uint16_t>(sizeof(uint32_t)));  // histogram record length
    poBuffer->AppendUInt32(HEADER_LENGTH +
                           RECORD_LENGTH);  // color/grayscale table offset
    poBuffer->AppendUInt32(HEADER_LENGTH + RECORD_LENGTH *
                                               CADRG_MAX_COLOR_ENTRY_COUNT *
                                               COLOR_TABLE_ENTRY_SIZE);

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

    // Reserve space for histogram. Will be patches in Patch_CADRG_Histogram()
    poBuffer->DeclareOffsetAtCurrentPosition("HISTOGRAM_LOCATION");
    for (int i = 0; i < CADRG_MAX_COLOR_ENTRY_COUNT; ++i)
    {
        poBuffer->AppendByte('?');
        poBuffer->AppendByte('?');
        poBuffer->AppendByte('?');
        poBuffer->AppendByte('?');
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
    bool bRet = Create_CADRG_CoverageSection(offsetPatcher, poSrcDS);
    Create_CADRG_ColorGrayscaleSection(offsetPatcher);
    Create_CADRG_ColormapSection(offsetPatcher, poSrcDS);
    Create_CADRG_ImageDescriptionSection(offsetPatcher, poSrcDS);
    return bRet;
}

/************************************************************************/
/*                    RPFFrameWriteCADRG_RPFIMG()                       */
/************************************************************************/

bool RPFFrameWriteCADRG_RPFIMG(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                               VSILFILE *fp, int &nUDIDL)
{
    std::vector<GDALOffsetPatcher::OffsetPatcherBuffer *> apoBuffers;
    int nContentLength = 0;
    for (const char *pszName :
         {"LocationComponent", "CoverageSectionSubheader",
          "ColorGrayscaleSectionSubheader", "ColormapSubsection",
          "ImageDescriptionSubheader"})
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
/*                     Write_CADRG_MaskSubsection()                     */
/************************************************************************/

static bool
Write_CADRG_MaskSubsection(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                           VSILFILE *fp)
{
    auto poBuffer = offsetPatcher->CreateBuffer(
        "MaskSubsection", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareBufferWrittenAtPosition(fp->Tell());
    poBuffer->DeclareOffsetAtCurrentPosition("MASK_SUBSECTION_LOCATION");
    poBuffer->AppendUInt16(0);  // SUBFRAME_SEQUENCE_RECORD_LENGTH
    poBuffer->AppendUInt16(0);  // TRANSPARENCY_SEQUENCE_RECORD_LENGTH
    poBuffer->AppendUInt16(0);  // TRANSPARENT_OUTPUT_PIXEL_CODE_LENGTH

    return fp->Write(poBuffer->GetBuffer().data(), 1,
                     poBuffer->GetBuffer().size()) ==
           poBuffer->GetBuffer().size();
}

/************************************************************************/
/*                   Write_CADRG_CompressionSection()                   */
/************************************************************************/

constexpr uint16_t NUMBER_OF_COMPRESSION_LOOKUP_OFFSET_RECORDS = 4;

static bool
Write_CADRG_CompressionSection(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                               VSILFILE *fp)
{
    auto poBuffer = offsetPatcher->CreateBuffer(
        "CompressionSection", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareBufferWrittenAtPosition(fp->Tell());
    poBuffer->DeclareOffsetAtCurrentPosition("COMPRESSION_SECTION_LOCATION");
    poBuffer->AppendUInt16(1);  // COMPRESSION_ALGORITHM_ID = VQ
    poBuffer->AppendUInt16(NUMBER_OF_COMPRESSION_LOOKUP_OFFSET_RECORDS);
    // NUMBER_OF_COMPRESSION_PARAMETER_OFFSET_RECORDS
    poBuffer->AppendUInt16(0);

    return fp->Write(poBuffer->GetBuffer().data(), 1,
                     poBuffer->GetBuffer().size()) ==
           poBuffer->GetBuffer().size();
}

/************************************************************************/
/*                  Write_CADRG_ImageDisplayParametersSection()         */
/************************************************************************/

static bool Write_CADRG_ImageDisplayParametersSection(
    GDALOffsetPatcher::OffsetPatcher *offsetPatcher, VSILFILE *fp)
{
    auto poBuffer = offsetPatcher->CreateBuffer(
        "ImageDisplayParametersSubheader", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBuffer);
    poBuffer->DeclareBufferWrittenAtPosition(fp->Tell());
    poBuffer->DeclareOffsetAtCurrentPosition(
        "IMAGE_DISPLAY_PARAMETERS_SECTION_LOCATION");
    poBuffer->AppendUInt32(BLOCK_SIZE / SUBSAMPLING);  // NUMBER_OF_IMAGE_ROWS
    poBuffer->AppendUInt32(BLOCK_SIZE /
                           SUBSAMPLING);  // NUMBER_OF_CODES_PER_ROW
    constexpr GByte CODE_WORD_BIT_LENGTH = 12;
    static_assert((1 << CODE_WORD_BIT_LENGTH) == CODEBOOK_MAX_SIZE);
    poBuffer->AppendByte(CODE_WORD_BIT_LENGTH);  // IMAGE_CODE_BIT_LENGTH

    return fp->Write(poBuffer->GetBuffer().data(), 1,
                     poBuffer->GetBuffer().size()) ==
           poBuffer->GetBuffer().size();
}

/************************************************************************/
/*                      Perform_CADRG_VQ_Compression()                  */
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
/*                  Write_CADRG_CompressionLookupSubSection()           */
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
/*                    Write_CADRG_SpatialDataSubsection()               */
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
/*                          Patch_CADRG_Histogram()                     */
/************************************************************************/

static bool Patch_CADRG_Histogram(
    GDALOffsetPatcher::OffsetPatcher *offsetPatcher, VSILFILE *fp,
    [[maybe_unused]] GDALDataset *poSrcDS,
    const std::vector<BucketItem<ColorTableBased4x4Pixels>> &codebook)
{
    auto poColormapBuffer =
        offsetPatcher->GetBufferFromName("ColormapSubsection");
    CPLAssert(poColormapBuffer);
    const auto nColormapFileLocation = poColormapBuffer->GetFileLocation();
    CPLAssert(nColormapFileLocation != GDALOffsetPatcher::INVALID_OFFSET);
    auto poHistogramLocDecl =
        offsetPatcher->GetOffsetDeclaration("HISTOGRAM_LOCATION");
    CPLAssert(poHistogramLocDecl);
    CPLAssert(poHistogramLocDecl->GetLocation().offsetInBuffer != 0);
    poHistogramLocDecl->MarkAsConsumed();

    // Compute the number of pixels in the output image per colormap entry
    std::vector<uint32_t> anHistogram(CADRG_MAX_COLOR_ENTRY_COUNT);
    const size_t nOffsetInBuffer =
        poHistogramLocDecl->GetLocation().offsetInBuffer;
    CPLAssert(nOffsetInBuffer + anHistogram.size() * sizeof(anHistogram[0]) ==
              poColormapBuffer->GetBuffer().size());
#ifdef DEBUG
    size_t nTotalCount = 0;
#endif
    for (const auto &item : codebook)
    {
        for (GByte byVal : item.m_vec.vals())
        {
            anHistogram[byVal] += item.m_count;
#ifdef DEBUG
            nTotalCount += item.m_count;
#endif
        }
    }
#ifdef DEBUG
    CPLAssert(nTotalCount == static_cast<size_t>(poSrcDS->GetRasterXSize()) *
                                 poSrcDS->GetRasterYSize());
#endif

#ifdef CPL_IS_LSB
    for (auto &nCount : anHistogram)
    {
        CPL_SWAP32PTR(&nCount);
    }
#endif

    return fp->Seek(nColormapFileLocation + nOffsetInBuffer, SEEK_SET) == 0 &&
           fp->Write(anHistogram.data(), sizeof(anHistogram[0]),
                     anHistogram.size()) == anHistogram.size();
}

/************************************************************************/
/*                      RPFFrameCreateCADRG_ImageContent()              */
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
           Patch_CADRG_Histogram(offsetPatcher, fp, poSrcDS, codebook) &&
           fp->Seek(0, SEEK_END) == 0 &&
           Write_CADRG_MaskSubsection(offsetPatcher, fp) &&
           Write_CADRG_CompressionSection(offsetPatcher, fp) &&
           Write_CADRG_ImageDisplayParametersSection(offsetPatcher, fp) &&
           Write_CADRG_CompressionLookupSubSection(offsetPatcher, fp,
                                                   codebook) &&
           Write_CADRG_SpatialDataSubsection(offsetPatcher, fp, poSrcDS,
                                             VQImage);
}

/************************************************************************/
/*                            RPFAttribute                              */
/************************************************************************/

namespace
{
struct RPFAttribute
{
    uint16_t nAttrId = 0;
    uint8_t nParamId = 0;
    std::string osValue{};

    RPFAttribute(int nAttrIdIn, int nParamIdIn, const std::string &osValueIn)
        : nAttrId(static_cast<uint16_t>(nAttrIdIn)),
          nParamId(static_cast<uint8_t>(nParamIdIn)), osValue(osValueIn)
    {
    }
};
}  // namespace

/************************************************************************/
/*                      RPFFrameWriteCADRG_RPFDES()                     */
/************************************************************************/

bool RPFFrameWriteCADRG_RPFDES(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                               VSILFILE *fp, vsi_l_offset nOffsetLDSH,
                               const CPLStringList &aosOptions)
{
    bool bOK = fp->Seek(0, SEEK_END) == 0;

    constexpr const char *pszDESHeader =
        "DE"                                           // Segment type
        "Registered Extensions    "                    // DESID
        "01"                                           // DESVER
        "U"                                            // DECLAS
        "  "                                           // DESCLSY
        "           "                                  // DESCODE
        "  "                                           // DESCTLH
        "                    "                         // DESREL
        "  "                                           // DESDCDT
        "        "                                     // DESDCDT
        "    "                                         // DESDCXM
        " "                                            // DESDG
        "        "                                     // DESDGDT
        "                                           "  // DESCLTX
        " "                                            // DESCATP
        "                                        "     // DESCAUT
        " "                                            // DESCRSN
        "        "                                     // DESSRDT
        "               "                              // DESCTLN
        "UDID  "                                       // DESOVFL
        "001"                                          // DESITEM
        "0000"                                         // DESSHL
        ;

    bOK &= fp->Write(pszDESHeader, 1, strlen(pszDESHeader)) ==
           strlen(pszDESHeader);

    std::string osDESData("RPFDES");
    std::string osDESDataPayload;
    const auto nPosAttributeSectionSubheader =
        fp->Tell() + osDESData.size() + strlen("XXXXX");

    auto poBufferASSH = offsetPatcher->CreateBuffer(
        "AttributeSectionSubheader", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBufferASSH);
    poBufferASSH->DeclareBufferWrittenAtPosition(nPosAttributeSectionSubheader);
    poBufferASSH->DeclareOffsetAtCurrentPosition(
        "ATTRIBUTE_SECTION_SUBHEADER_LOCATION");

    std::vector<RPFAttribute> asAttributes{
        // Horizontal datum code
        {7, 1, "WGE " /* trailing space intended */},
    };

    if (const char *pszV =
            aosOptions.FetchNameValue("RPF_DataSeriesDesignation"))
    {
        asAttributes.emplace_back(4, 1, StrPadTruncate(pszV, 10));
    }

    if (const char *pszV = aosOptions.FetchNameValue("RPF_MapDesignation"))
    {
        asAttributes.emplace_back(4, 2, StrPadTruncate(pszV, 8));
    }

    const auto nAttrCount = static_cast<uint16_t>(asAttributes.size());
    poBufferASSH->AppendUInt16(nAttrCount);
    poBufferASSH->AppendUInt16(0);  // NUMBER_OF_EXPLICIT_AREAL_COVERAGE_RECORDS
    poBufferASSH->AppendUInt32(0);  // ATTRIBUTE_OFFSET_TABLE_OFFSET
    constexpr uint16_t ATTRIBUTE_OFFSET_RECORD_LENGTH =
        static_cast<uint16_t>(8);
    poBufferASSH->AppendUInt16(ATTRIBUTE_OFFSET_RECORD_LENGTH);

    osDESDataPayload.insert(
        osDESDataPayload.end(),
        reinterpret_cast<const char *>(poBufferASSH->GetBuffer().data()),
        reinterpret_cast<const char *>(poBufferASSH->GetBuffer().data() +
                                       poBufferASSH->GetBuffer().size()));

    auto poBufferAS = offsetPatcher->CreateBuffer(
        "AttributeSubsection", /* bEndiannessIsLittle = */ false);
    CPLAssert(poBufferAS);
    poBufferAS->DeclareBufferWrittenAtPosition(
        nPosAttributeSectionSubheader + poBufferASSH->GetBuffer().size());
    poBufferAS->DeclareOffsetAtCurrentPosition("ATTRIBUTE_SUBSECTION_LOCATION");

    size_t nAttrValueOffset =
        ATTRIBUTE_OFFSET_RECORD_LENGTH * asAttributes.size();

    // Attribute definitions
    for (const auto &sAttr : asAttributes)
    {
        poBufferAS->AppendUInt16(sAttr.nAttrId);
        poBufferAS->AppendByte(sAttr.nParamId);
        poBufferAS->AppendByte(0);  // Areal coverage sequence number
        poBufferAS->AppendUInt32(static_cast<uint32_t>(
            nAttrValueOffset));  // Attribute record offset
        nAttrValueOffset += sAttr.osValue.size();
    }

    // Attribute values
    for (const auto &sAttr : asAttributes)
    {
        poBufferAS->AppendString(sAttr.osValue);
    }

    osDESDataPayload.insert(
        osDESDataPayload.end(),
        reinterpret_cast<const char *>(poBufferAS->GetBuffer().data()),
        reinterpret_cast<const char *>(poBufferAS->GetBuffer().data() +
                                       poBufferAS->GetBuffer().size()));

    CPLAssert(osDESDataPayload.size() <= 99999U);
    osDESData += CPLSPrintf("%05d", static_cast<int>(osDESDataPayload.size()));
    osDESData += osDESDataPayload;
    bOK &=
        fp->Write(osDESData.c_str(), 1, osDESData.size()) == osDESData.size();

    // Update LDSH and LD in the NITF Header
    const int iDES = 0;
    bOK &= fp->Seek(nOffsetLDSH + iDES * 13, SEEK_SET) == 0;
    bOK &= fp->Write(CPLSPrintf("%04d", static_cast<int>(strlen(pszDESHeader))),
                     1, 4) == 4;
    bOK &= fp->Write(CPLSPrintf("%09d", static_cast<int>(osDESData.size())), 1,
                     9) == 9;

    return bOK;
}

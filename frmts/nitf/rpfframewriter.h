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

#ifndef RPFFRAME_WRITER_INCLUDED
#define RPFFRAME_WRITER_INCLUDED

#include "cpl_string.h"
#include "cpl_vsi_virtual.h"
#include <string>
#include <variant>

namespace GDALOffsetPatcher
{
class OffsetPatcher;
}

class GDALDataset;

constexpr int CADRG_MAX_COLOR_ENTRY_COUNT = 216;
constexpr int CADRG_FRAME_PIXEL_COUNT = 1536;

constexpr int Kilo = 1000;
constexpr int Million = Kilo * Kilo;

/** Opaque class containing CADRG related information */
class CADRGInformation
{
  public:
    class Private;
    explicit CADRGInformation(std::unique_ptr<Private>);
    ~CADRGInformation();

    bool HasTransparentPixels() const;

    std::unique_ptr<Private> m_private{};
};

void Create_CADRG_RPFHDR(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                         const std::string &osFilename,
                         CPLStringList &aosOptions);

std::unique_ptr<CADRGInformation>
RPFFrameCreateCADRG_TREs(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                         const std::string &osFilename, GDALDataset *poSrcDS,
                         CPLStringList &aosOptions, int nReciprocalScale);

bool RPFFrameWriteCADRG_RPFIMG(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                               VSILFILE *fp, int &nUDIDL);

bool RPFFrameWriteCADRG_ImageContent(
    GDALOffsetPatcher::OffsetPatcher *offsetPatcher, VSILFILE *fp,
    GDALDataset *poSrcDS, CADRGInformation *info);

const char *RPFFrameWriteGetDESHeader();

bool RPFFrameWriteCADRG_RPFDES(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                               VSILFILE *fp, vsi_l_offset nOffsetLDSH,
                               const CPLStringList &aosOptions,
                               int nReciprocalScale);

int RPFGetCADRGClosestReciprocalScale(GDALDataset *poSrcDS,
                                      double dfDPIOverride, bool &bGotDPI);

std::variant<bool, std::unique_ptr<GDALDataset>>
CADRGCreateCopy(const char *pszFilename, GDALDataset *poSrcDS, int bStrict,
                CSLConstList papszOptions, GDALProgressFunc pfnProgress,
                void *pProgressData, int nRecLevel, int &nReciprocalScale);

struct RPFFrameDef
{
    int nZone;
    int nReciprocalScale;
    int nFrameMinX;
    int nFrameMinY;
    int nFrameMaxX;
    int nFrameMaxY;
    double dfResX;
    double dfResY;
};

void RPFGetCADRGResolutionAndInterval(int nZone, int nReciprocalScale,
                                      double &latResolution,
                                      double &lonResolution,
                                      double &latInterval, double &lonInterval);
std::vector<RPFFrameDef>
RPFGetCADRGFramesForEnvelope(int nZoneIn, int nReciprocalScale, double dfXMin,
                             double dfYMin, double dfXMax, double dfYMax);

bool RPFGetCADRGFrameExtent(int nZone, int nReciprocalScale, int nFrameX,
                            int nFrameY, double &dfXMin, double &dfYMin,
                            double &dfXMax, double &dfYMax);

bool RPFCADRGIsKnownDataSeriesCode(const char *pszCode);

char RPFCADRGZoneNumToChar(int nZone);

int RPFCADRGZoneCharToNum(char chZone);

int RPFCADRGGetScaleFromDataSeriesCode(const char *pszCode);

std::string RPFGetCADRGFrameNumberAsString(int nZone, int nReciprocalScale,
                                           int nFrameX, int nFrameY);

#endif  // RPFFRAME_WRITER_INCLUDED

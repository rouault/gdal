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

namespace GDALOffsetPatcher
{
class OffsetPatcher;
}

class GDALDataset;

constexpr int CADRG_MAX_COLOR_ENTRY_COUNT = 216;

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

std::unique_ptr<CADRGInformation>
RPFFrameCreateCADRG_TREs(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                         const std::string &osFilename, GDALDataset *poSrcDS,
                         CPLStringList &aosOptions);

bool RPFFrameWriteCADRG_RPFIMG(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                               VSILFILE *fp, int &nUDIDL);

bool RPFFrameWriteCADRG_ImageContent(
    GDALOffsetPatcher::OffsetPatcher *offsetPatcher, VSILFILE *fp,
    GDALDataset *poSrcDS, CADRGInformation *info);

bool RPFFrameWriteCADRG_RPFDES(GDALOffsetPatcher::OffsetPatcher *offsetPatcher,
                               VSILFILE *fp, vsi_l_offset nOffsetLDSH,
                               const CPLStringList &aosOptions);

#endif  // RPFFRAME_WRITER_INCLUDED

/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Private definitions within the Shapefile driver to implement
 *           integration with OGR.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 1999,  Les Technologies SoftMap Inc.
 * Copyright (c) 2008-2013, Even Rouault <even dot rouault at spatialys.com>
 *
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

#ifndef OGRMIRAMON_H_INCLUDED
#define OGRMIRAMON_H_INCLUDED

#include "ogrsf_frmts.h"
#include "ogr_api.h"
#include "cpl_string.h"
#include "mm_wrlayr.h"

/************************************************************************/
/*                             OGRMiraMonLayer                              */
/************************************************************************/

class OGRMiraMonLayer final : public OGRLayer,
                          public OGRGetNextFeatureThroughRaw<OGRMiraMonLayer>
{
    OGRSpatialReference *m_poSRS = nullptr;
    OGRFeatureDefn *poFeatureDefn;

    GUIntBig iNextFID;

    struct MiraMonVectLayerInfo hMiraMonLayer; // MiraMon layer
    struct MM_TH pMMHeader; // Main header
    struct MiraMonFeature hMMFeature; // Feature reading/writing
    struct MiraMonDataBase hLayerDB;

    bool bUpdate;
    
    VSILFILE *m_fp = nullptr;

    CPLString osLine;
    char **papszKeyedValues;

    OGRFeature *GetNextRawFeature();
    void OGRMiraMonLayer::GoToFieldOfMultipleRecord(MM_INTERNAL_FID iFID,
                    MM_EXT_DBF_N_RECORDS nIRecord, MM_EXT_DBF_N_FIELDS nIField);

    OGRErr OGRMiraMonLayer::DumpVertices(OGRGeometryH hGeom,
                    bool bExternalRing, int eLT);
    OGRErr OGRMiraMonLayer::LoadGeometry(OGRGeometryH hGeom,
                                        bool bExternalRing,
                                        OGRFeature *poFeature);
    OGRErr WriteGeometry(bool bExternalRing, 
                    OGRFeature *poFeature);
    GIntBig GetFeatureCount(int bForce);
    
  public:
    bool bValidFile;

    OGRMiraMonLayer(const char *pszFilename, VSILFILE *fp,
                const OGRSpatialReference *poSRS, int bUpdate,
                char **papszOpenOptions);
    virtual ~OGRMiraMonLayer();

    void ResetReading() override;
    DEFINE_GET_NEXT_FEATURE_THROUGH_RAW(OGRMiraMonLayer)

    OGRFeatureDefn *GetLayerDefn() override
    {
        return poFeatureDefn;
    }
    
    OGRErr TranslateFieldsToMM();
    OGRErr TranslateFieldsValuesToMM(OGRFeature *poFeature);
    OGRErr GetExtent(OGREnvelope *psExtent, int bForce) override;
    virtual OGRErr GetExtent(int iGeomField, OGREnvelope *psExtent,
                             int bForce) override
    {
        return OGRLayer::GetExtent(iGeomField, psExtent, bForce);
    }

    OGRErr ICreateFeature(OGRFeature *poFeature) override;

    virtual OGRErr CreateField(const OGRFieldDefn *poField,
                               int bApproxOK = TRUE) ;

    int TestCapability(const char *) override;
};

/************************************************************************/
/*                           OGRMiraMonDataSource                           */
/************************************************************************/

class OGRMiraMonDataSource final : public OGRDataSource
{
    OGRMiraMonLayer **papoLayers;
    int nLayers;

    char *pszName;

    bool bUpdate;
    
  public:
    OGRMiraMonDataSource();
    virtual ~OGRMiraMonDataSource();

    int Open(const char *pszFilename, VSILFILE *fp,
             const OGRSpatialReference *poSRS, int bUpdate,
            char **papszOpenOptions);
    int Create(const char *pszFilename, char **papszOptions);

    const char *GetName() override
    {
        return pszName;
    }
    int GetLayerCount() override
    {
        return nLayers;
    }
    OGRLayer *GetLayer(int) override;

    virtual OGRLayer *ICreateLayer(const char *,
                                   const OGRSpatialReference * = nullptr,
                                   OGRwkbGeometryType = wkbUnknown,
                                   char ** = nullptr) override;

    int TestCapability(const char *) override;
};


#endif /* ndef OGRMIRAMON_H_INCLUDED */

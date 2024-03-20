/******************************************************************************
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Implements OGRMiraMonLayer class.
 * Author:   Abel Pau
 ******************************************************************************
 * Copyright (c) 2024, Xavier Pons
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
#include "ogrmiramon.h"

#include "mm_gdal_functions.h"  // For MMCreateExtendedDBFIndex()
#include "mm_gdal_constants.h"  // For strcasecmp()
#include "mm_rdlayr.h"          // For MMInitLayerToRead()

/****************************************************************************/
/*                            OGRMiraMonLayer()                             */
/****************************************************************************/
OGRMiraMonLayer::OGRMiraMonLayer(const char *pszFilename, VSILFILE *fp,
                                 const OGRSpatialReference *poSRS,
                                 int bUpdateIn, CSLConstList papszOpenOptions,
                                 struct MiraMonVectMapInfo *MMMap)
    : poFeatureDefn(nullptr), iNextFID(0), phMiraMonLayer(nullptr),
      hMiraMonLayerPNT(), hMiraMonLayerARC(), hMiraMonLayerPOL(),
      hMiraMonLayerReadOrNonGeom(), hMMFeature(),
      bUpdate(CPL_TO_BOOL(bUpdateIn)), nMMMemoryRatio(1.0),
      m_fp(fp ? fp : VSIFOpenL(pszFilename, (bUpdateIn ? "r+" : "r"))),
      papszValues(nullptr), padfValues(nullptr), bValidFile(false)
{

    CPLDebug("MiraMon", "Creating/Opening MiraMon layer...");
    /* -------------------------------------------------------------------- */
    /*      Create the feature definition                                   */
    /* -------------------------------------------------------------------- */
    poFeatureDefn = new OGRFeatureDefn(CPLGetBasename(pszFilename));
    SetDescription(poFeatureDefn->GetName());
    poFeatureDefn->Reference();

    if (bUpdate)
    {
        /* ---------------------------------------------------------------- */
        /*      Establish the version to use                                */
        /* ---------------------------------------------------------------- */
        const char *pszVersion = CSLFetchNameValue(papszOpenOptions, "Version");
        int nMMVersion;

        if (pszVersion)
        {
            if (EQUAL(pszVersion, "V1.1"))
                nMMVersion = MM_32BITS_VERSION;
            else if (EQUAL(pszVersion, "V2.0") ||
                     EQUAL(pszVersion, "last_version"))
                nMMVersion = MM_64BITS_VERSION;
            else
                nMMVersion = MM_32BITS_VERSION;  // Default
        }
        else
            nMMVersion = MM_32BITS_VERSION;  // Default

        /* ---------------------------------------------------------------- */
        /*      Establish the charset of the .dbf files                     */
        /* ---------------------------------------------------------------- */
        const char *pszdbfEncoding =
            CSLFetchNameValue(papszOpenOptions, "DBFEncoding");
        char nMMRecode;

        if (pszdbfEncoding)
        {
            if (EQUAL(pszdbfEncoding, "UTF8"))
                nMMRecode = MM_RECODE_UTF8;
            else  //if (EQUAL(pszdbfEncoding, "ANSI"))
                nMMRecode = MM_RECODE_ANSI;
        }
        else
            nMMRecode = MM_RECODE_ANSI;  // Default

        /* ----------------------------------------------------------------- */
        /*      Establish the nMemoryRatio to use                            */
        /* ----------------------------------------------------------------- */
        const char *pszMemoryRatio =
            CSLFetchNameValue(papszOpenOptions, "CreationMemoryRatio");

        if (pszMemoryRatio)
            nMMMemoryRatio = atof(pszMemoryRatio);
        else
            nMMMemoryRatio = 1;  // Default

        /* ----------------------------------------------------------------- */
        /*   Establish the descriptors language when                         */
        /*   creating .rel files                                             */
        /* ----------------------------------------------------------------- */
        const char *pszLanguage =
            CSLFetchNameValue(papszOpenOptions, "CreationLanguage");
        char nMMLanguage;

        if (pszLanguage)
        {
            if (EQUAL(pszLanguage, "CAT"))
                nMMLanguage = MM_CAT_LANGUAGE;
            else if (EQUAL(pszLanguage, "SPA"))
                nMMLanguage = MM_SPA_LANGUAGE;
            else
                nMMLanguage = MM_ENG_LANGUAGE;
        }
        else
            nMMLanguage = MM_ENG_LANGUAGE;  // Default

        /* ---------------------------------------------------------------- */
        /*      Preparing to write the layer                                */
        /* ---------------------------------------------------------------- */
        if (!STARTS_WITH(pszFilename, "/vsistdout"))
        {
            // Init the feature (memory, num,...)
            if (MMInitFeature(&hMMFeature))
            {
                bValidFile = false;
                return;
            }

            // Init the Layers (not in disk, only in memory until
            // the first element is readed)
            CPLDebug("MiraMon", "Initializing MiraMon points layer...");
            if (MMInitLayer(&hMiraMonLayerPNT, pszFilename, nMMVersion,
                            nMMRecode, nMMLanguage, nMMMemoryRatio, nullptr,
                            MM_WRITTING_MODE, MMMap))
            {
                bValidFile = false;
                return;
            }
            hMiraMonLayerPNT.bIsBeenInit = 0;

            CPLDebug("MiraMon", "Initializing MiraMon arcs layer...");
            if (MMInitLayer(&hMiraMonLayerARC, pszFilename, nMMVersion,
                            nMMRecode, nMMLanguage, nMMMemoryRatio, nullptr,
                            MM_WRITTING_MODE, MMMap))
            {
                bValidFile = false;
                return;
            }
            hMiraMonLayerARC.bIsBeenInit = 0;

            CPLDebug("MiraMon", "Initializing MiraMon polygons layer...");
            if (MMInitLayer(&hMiraMonLayerPOL, pszFilename, nMMVersion,
                            nMMRecode, nMMLanguage, nMMMemoryRatio, nullptr,
                            MM_WRITTING_MODE, MMMap))
            {
                bValidFile = false;
                return;
            }
            hMiraMonLayerPOL.bIsBeenInit = 0;

            // Just in case that there is no geometry but some other
            // information to get. A DBF will be generated
            CPLDebug("MiraMon", "Initializing MiraMon only-ext-DBF layer...");
            if (MMInitLayer(&hMiraMonLayerReadOrNonGeom, pszFilename,
                            nMMVersion, nMMRecode, nMMLanguage, nMMMemoryRatio,
                            nullptr, MM_WRITTING_MODE, nullptr))
            {
                bValidFile = false;
                return;
            }
            hMiraMonLayerPOL.bIsBeenInit = 0;

            // This helps the map to be created
            //GetLayerDefn()->SetName(hMiraMonLayerPNT.pszSrcLayerName);
            poFeatureDefn->SetName(hMiraMonLayerPNT.pszSrcLayerName);
        }

        // Saving the HRS in the layer structure
        if (poSRS)
        {
            if (poSRS->GetAuthorityName(nullptr) &&
                EQUAL(poSRS->GetAuthorityName(nullptr), "EPSG"))
            {
                CPLDebug("MiraMon", "Setting EPSG code %s",
                         poSRS->GetAuthorityCode(nullptr));
                hMiraMonLayerPNT.pSRS =
                    CPLStrdup(poSRS->GetAuthorityCode(nullptr));
                hMiraMonLayerARC.pSRS =
                    CPLStrdup(poSRS->GetAuthorityCode(nullptr));
                hMiraMonLayerPOL.pSRS =
                    CPLStrdup(poSRS->GetAuthorityCode(nullptr));
            }
        }
    }
    else
    {
        if (m_fp == nullptr)
        {
            bValidFile = false;
            return;
        }

        /* ------------------------------------------------------------------*/
        /*      Read the header.                                             */
        /* ------------------------------------------------------------------*/
        if (!STARTS_WITH(pszFilename, "/vsistdout"))
        {
            int nMMLayerVersion;

            if (MMInitLayerToRead(&hMiraMonLayerReadOrNonGeom, m_fp,
                                  pszFilename))
            {
                phMiraMonLayer = &hMiraMonLayerReadOrNonGeom;
                bValidFile = false;
                return;
            }
            phMiraMonLayer = &hMiraMonLayerReadOrNonGeom;

            nMMLayerVersion = MMGetVectorVersion(&phMiraMonLayer->TopHeader);
            if (nMMLayerVersion == MM_UNKNOWN_VERSION)
            {
                CPLError(CE_Failure, CPLE_NotSupported,
                         "MiraMon version file unknown.");
                bValidFile = false;
                return;
            }
            if (phMiraMonLayer->bIsPoint)
            {
                if (phMiraMonLayer->TopHeader.bIs3d)
                    poFeatureDefn->SetGeomType(wkbPoint25D);
                else
                    poFeatureDefn->SetGeomType(wkbPoint);
            }
            else if (phMiraMonLayer->bIsArc && !phMiraMonLayer->bIsPolygon)
            {
                if (phMiraMonLayer->TopHeader.bIs3d)
                    poFeatureDefn->SetGeomType(wkbLineString25D);
                else
                    poFeatureDefn->SetGeomType(wkbLineString);
            }
            else if (phMiraMonLayer->bIsPolygon)
            {
                // 3D
                if (phMiraMonLayer->TopHeader.bIs3d)
                {
                    if (phMiraMonLayer->TopHeader.bIsMultipolygon)
                        poFeatureDefn->SetGeomType(wkbMultiPolygon25D);
                    else
                        poFeatureDefn->SetGeomType(wkbPolygon25D);
                }
                else
                {
                    if (phMiraMonLayer->TopHeader.bIsMultipolygon)
                        poFeatureDefn->SetGeomType(wkbMultiPolygon);
                    else
                        poFeatureDefn->SetGeomType(wkbPolygon);
                }
            }
            else
            {
                CPLError(CE_Failure, CPLE_NotSupported,
                         "MiraMon file type not supported.");
                bValidFile = false;
                return;
            }

            if (phMiraMonLayer->TopHeader.bIs3d)
            {
                const char *szHeight =
                    CSLFetchNameValue(papszOpenOptions, "Height");
                if (szHeight)
                {
                    if (!strcasecmp(szHeight, "Highest"))
                        phMiraMonLayer->nSelectCoordz =
                            MM_SELECT_HIGHEST_COORDZ;
                    else if (!strcasecmp(szHeight, "Lowest"))
                        phMiraMonLayer->nSelectCoordz = MM_SELECT_LOWEST_COORDZ;
                    else
                        phMiraMonLayer->nSelectCoordz = MM_SELECT_FIRST_COORDZ;
                }
                else
                    phMiraMonLayer->nSelectCoordz = MM_SELECT_FIRST_COORDZ;
            }

            /* ------------------------------------------------------------ */
            /*      Establish the nMemoryRatio to use                       */
            /* ------------------------------------------------------------ */
            const char *pszMemoryRatio =
                CSLFetchNameValue(papszOpenOptions, "OpenMemoryRatio");

            if (pszMemoryRatio)
                phMiraMonLayer->nMemoryRatio = atof(pszMemoryRatio);
            else
                phMiraMonLayer->nMemoryRatio = 1;  // Default

            if (phMiraMonLayer->nMemoryRatio < 0)
                phMiraMonLayer->nMemoryRatio =
                    fabs(phMiraMonLayer->nMemoryRatio);

            // It has no sense try to work with less ratio than 0.25.
            if (phMiraMonLayer->nMemoryRatio < 0.25)
                phMiraMonLayer->nMemoryRatio = 0.25;

            /* ------------------------------------------------------------ */
            /*   Establish the descriptors language when                    */
            /*   opening .rel files                                        */
            /* ------------------------------------------------------------ */
            const char *pszLanguage =
                CSLFetchNameValue(papszOpenOptions, "OpenLanguage");

            if (pszLanguage)
            {
                if (EQUAL(pszLanguage, "CAT"))
                    phMiraMonLayer->nMMLanguage = MM_CAT_LANGUAGE;
                else if (EQUAL(pszLanguage, "SPA"))
                    phMiraMonLayer->nMMLanguage = MM_SPA_LANGUAGE;
                else
                    phMiraMonLayer->nMMLanguage = MM_ENG_LANGUAGE;
            }
            else
                phMiraMonLayer->nMMLanguage = MM_ENG_LANGUAGE;  // Default

            if (phMiraMonLayer->nSRS_EPSG != 0)
            {
                m_poSRS = new OGRSpatialReference();
                m_poSRS->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
                if (m_poSRS->importFromEPSG(phMiraMonLayer->nSRS_EPSG) !=
                    OGRERR_NONE)
                {
                    delete m_poSRS;
                    m_poSRS = nullptr;
                }
            }

            // If there is associated information
            if (phMiraMonLayer->pMMBDXP && phMiraMonLayer->pMMBDXP->nRecords)
            {
                if (!phMiraMonLayer->pMMBDXP->pfDataBase)
                {
                    if ((phMiraMonLayer->pMMBDXP->pfDataBase =
                             fopen_function(phMiraMonLayer->pMMBDXP->szFileName,
                                            "r")) == nullptr)
                    {
                        CPLDebug("MiraMon", "File '%s' cannot be opened.",
                                 phMiraMonLayer->pMMBDXP->szFileName);
                        bValidFile = false;
                        return;
                    }

                    // First time we open the extended DBF we create an index
                    // to fastly find all non geometrical features.
                    phMiraMonLayer->pMultRecordIndex = MMCreateExtendedDBFIndex(
                        phMiraMonLayer->pMMBDXP->pfDataBase,
                        phMiraMonLayer->pMMBDXP->nRecords,
                        phMiraMonLayer->pMMBDXP->FirstRecordOffset,
                        phMiraMonLayer->pMMBDXP->BytesPerRecord,
                        phMiraMonLayer->pMMBDXP
                            ->pField[phMiraMonLayer->pMMBDXP->IdGraficField]
                            .AcumulatedBytes,
                        phMiraMonLayer->pMMBDXP
                            ->pField[phMiraMonLayer->pMMBDXP->IdGraficField]
                            .BytesPerField,
                        &phMiraMonLayer->isListField, &phMiraMonLayer->nMaxN);

                    if (!phMiraMonLayer->pMultRecordIndex)
                    {
                        CPLDebug("MiraMon", "File '%s' cannot be read.",
                                 phMiraMonLayer->pMMBDXP->szFileName);
                        bValidFile = false;
                        return;
                    }
                    // Creation of maximum number needed for processing
                    // multiple records
                    papszValues = static_cast<char **>(
                        CPLCalloc((size_t)phMiraMonLayer->nMaxN + 1,
                                  sizeof(*papszValues)));

                    padfValues = static_cast<double *>(CPLCalloc(
                        (size_t)phMiraMonLayer->nMaxN, sizeof(*padfValues)));

                    phMiraMonLayer->iMultiRecord =
                        -2;  // No option iMultiRecord
                    const char *szMultiRecord =
                        CSLFetchNameValue(papszOpenOptions, "iMultiRecord");
                    if (phMiraMonLayer->isListField && szMultiRecord)
                    {
                        if (!strcasecmp(szMultiRecord, "Last"))
                            phMiraMonLayer->iMultiRecord = -1;
                        else if (!strcasecmp(szMultiRecord, "JSON"))
                            phMiraMonLayer->iMultiRecord = -3;
                        else
                            phMiraMonLayer->iMultiRecord = atoi(szMultiRecord);
                    }
                }

                for (MM_EXT_DBF_N_FIELDS nIField = 0;
                     nIField < phMiraMonLayer->pMMBDXP->nFields; nIField++)
                {
                    OGRFieldDefn oField("", OFTString);
                    oField.SetName(
                        phMiraMonLayer->pMMBDXP->pField[nIField].FieldName);

                    if (phMiraMonLayer->pMMBDXP->pField[nIField].FieldType ==
                        'C')
                    {
                        // It's a list?
                        if (phMiraMonLayer->iMultiRecord == -2)
                        {
                            if (phMiraMonLayer->isListField)
                                oField.SetType(OFTStringList);
                            else
                                oField.SetType(OFTString);
                        }
                        // It's a serialized JSON array
                        else if (phMiraMonLayer->iMultiRecord == -3)
                        {
                            oField.SetType(OFTString);
                            oField.SetSubType(OFSTJSON);
                        }
                        else  // iMultiRecord decides which Record translate
                            oField.SetType(OFTString);
                    }
                    else if (phMiraMonLayer->pMMBDXP->pField[nIField]
                                 .FieldType == 'N')
                    {
                        // It's a list?
                        if (phMiraMonLayer->iMultiRecord == -2)
                        {
                            if (phMiraMonLayer->pMMBDXP->pField[nIField]
                                    .DecimalsIfFloat)
                                oField.SetType(phMiraMonLayer->isListField
                                                   ? OFTRealList
                                                   : OFTReal);
                            else
                                oField.SetType(phMiraMonLayer->isListField
                                                   ? OFTIntegerList
                                                   : OFTInteger);
                        }
                        // It's a serialized JSON array
                        else if (phMiraMonLayer->iMultiRecord == -3)
                        {
                            oField.SetType(OFTString);
                            oField.SetSubType(OFSTJSON);
                        }
                        else
                        {
                            if (phMiraMonLayer->pMMBDXP->pField[nIField]
                                    .DecimalsIfFloat)
                                oField.SetType(OFTReal);
                            else
                                oField.SetType(OFTInteger);
                        }
                    }
                    else if (phMiraMonLayer->pMMBDXP->pField[nIField]
                                 .FieldType == 'D')
                    {
                        // It's a serialized JSON array
                        oField.SetType(OFTDate);
                        if (phMiraMonLayer->iMultiRecord == -3)
                        {
                            oField.SetType(OFTString);
                            oField.SetSubType(OFSTJSON);
                        }
                    }

                    oField.SetWidth(
                        phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField);
                    oField.SetPrecision(phMiraMonLayer->pMMBDXP->pField[nIField]
                                            .DecimalsIfFloat);

                    poFeatureDefn->AddFieldDefn(&oField);
                }
            }
        }

        if (poSRS)
        {
            m_poSRS = poSRS->Clone();
            m_poSRS->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
        }

        poFeatureDefn->GetGeomFieldDefn(0)->SetSpatialRef(m_poSRS);
    }

    bValidFile = true;
}

/****************************************************************************/
/*                           ~OGRMiraMonLayer()                             */
/****************************************************************************/

OGRMiraMonLayer::~OGRMiraMonLayer()

{
    if (m_nFeaturesRead > 0 && poFeatureDefn != nullptr)
    {
        CPLDebug("MiraMon", "%d features read on layer '%s'.",
                 static_cast<int>(m_nFeaturesRead), poFeatureDefn->GetName());
    }

    if (hMiraMonLayerPOL.bIsPolygon)
    {
        CPLDebug("MiraMon", "Closing MiraMon polygons layer...");
        if (MMCloseLayer(&hMiraMonLayerPOL))
            CPLDebug("MiraMon", "Error closing polygons layer");
        if (hMiraMonLayerPOL.TopHeader.nElemCount)
        {
            CPLDebug("MiraMon",
                     sprintf_UINT64 " polygon(s) written in file %s.pol",
                     hMiraMonLayerPOL.TopHeader.nElemCount,
                     hMiraMonLayerPOL.pszSrcLayerName);
        }
        CPLDebug("MiraMon", "MiraMon polygons layer closed");
    }
    else if (hMiraMonLayerPOL.ReadOrWrite == MM_WRITTING_MODE)
        CPLDebug("MiraMon", "No MiraMon polygons layer created.");

    if (hMiraMonLayerARC.bIsArc)
    {
        CPLDebug("MiraMon", "Closing MiraMon arcs layer...");
        if (MMCloseLayer(&hMiraMonLayerARC))
            CPLDebug("MiraMon", "Error closing arcs layer");
        if (hMiraMonLayerARC.TopHeader.nElemCount)
        {
            CPLDebug("MiraMon", sprintf_UINT64 " arc(s) written in file %s.arc",
                     hMiraMonLayerARC.TopHeader.nElemCount,
                     hMiraMonLayerARC.pszSrcLayerName);
        }

        CPLDebug("MiraMon", "MiraMon arcs layer closed");
    }
    else if (hMiraMonLayerARC.ReadOrWrite == MM_WRITTING_MODE)
        CPLDebug("MiraMon", "No MiraMon arcs layer created.");

    if (hMiraMonLayerPNT.bIsPoint)
    {
        CPLDebug("MiraMon", "Closing MiraMon points layer...");
        if (MMCloseLayer(&hMiraMonLayerPNT))
            CPLDebug("MiraMon", "Error closing points layer");
        if (hMiraMonLayerPNT.TopHeader.nElemCount)
        {
            CPLDebug("MiraMon",
                     sprintf_UINT64 " point(s) written in file %s.pnt",
                     hMiraMonLayerPNT.TopHeader.nElemCount,
                     hMiraMonLayerPNT.pszSrcLayerName);
        }
        CPLDebug("MiraMon", "MiraMon points layer closed");
    }
    else if (hMiraMonLayerPNT.ReadOrWrite == MM_WRITTING_MODE)
        CPLDebug("MiraMon", "No MiraMon points layer created.");

    if (hMiraMonLayerARC.ReadOrWrite == MM_WRITTING_MODE)
    {
        if (hMiraMonLayerReadOrNonGeom.bIsDBF)
        {
            if (hMiraMonLayerReadOrNonGeom.ReadOrWrite == MM_WRITTING_MODE)
                CPLDebug("MiraMon", "Closing MiraMon DBF table ...");
            MMCloseLayer(&hMiraMonLayerReadOrNonGeom);
            if (hMiraMonLayerReadOrNonGeom.ReadOrWrite == MM_WRITTING_MODE)
                CPLDebug("MiraMon", "MiraMon DBF table closed");
        }
        else if (hMiraMonLayerReadOrNonGeom.ReadOrWrite == MM_WRITTING_MODE)
            CPLDebug("MiraMon", "No MiraMon DBF table created.");
    }
    else
    {
        if (hMiraMonLayerReadOrNonGeom.ReadOrWrite == MM_WRITTING_MODE)
            CPLDebug("MiraMon", "Closing MiraMon layer ...");
        MMCloseLayer(&hMiraMonLayerReadOrNonGeom);
        if (hMiraMonLayerReadOrNonGeom.ReadOrWrite == MM_WRITTING_MODE)
            CPLDebug("MiraMon", "MiraMon layer closed");
    }

    if (hMiraMonLayerPOL.ReadOrWrite == MM_WRITTING_MODE)
        MMCPLDebug("MiraMon", "Destroying MiraMon polygons layer memory");
    MMDestroyLayer(&hMiraMonLayerPOL);
    if (hMiraMonLayerPOL.ReadOrWrite == MM_WRITTING_MODE)
        MMCPLDebug("MiraMon", "MiraMon polygons layer memory destroyed");

    if (hMiraMonLayerARC.ReadOrWrite == MM_WRITTING_MODE)
        MMCPLDebug("MiraMon", "Destroying MiraMon arcs layer memory");
    MMDestroyLayer(&hMiraMonLayerARC);
    if (hMiraMonLayerARC.ReadOrWrite == MM_WRITTING_MODE)
        MMCPLDebug("MiraMon", "MiraMon arcs layer memory destroyed");

    if (hMiraMonLayerPNT.ReadOrWrite == MM_WRITTING_MODE)
        MMCPLDebug("MiraMon", "Destroying MiraMon points layer memory");
    MMDestroyLayer(&hMiraMonLayerPNT);
    if (hMiraMonLayerPNT.ReadOrWrite == MM_WRITTING_MODE)
        MMCPLDebug("MiraMon", "MiraMon points layer memory destroyed");

    if (hMiraMonLayerReadOrNonGeom.ReadOrWrite == MM_WRITTING_MODE)
        MMCPLDebug("MiraMon", "Destroying MiraMon DBF table layer memory");
    else
        MMCPLDebug("MiraMon", "Destroying MiraMon layer memory");

    MMDestroyLayer(&hMiraMonLayerReadOrNonGeom);
    if (hMiraMonLayerReadOrNonGeom.ReadOrWrite == MM_WRITTING_MODE)
        MMCPLDebug("MiraMon", "MiraMon DBF table layer memory destroyed");
    else
        MMCPLDebug("MiraMon", "MiraMon layer memory destroyed");

    memset(&hMiraMonLayerReadOrNonGeom, 0, sizeof(hMiraMonLayerReadOrNonGeom));
    memset(&hMiraMonLayerPNT, 0, sizeof(hMiraMonLayerPNT));
    memset(&hMiraMonLayerARC, 0, sizeof(hMiraMonLayerARC));
    memset(&hMiraMonLayerPOL, 0, sizeof(hMiraMonLayerPOL));

    MMCPLDebug("MiraMon", "Destroying MiraMon temporary feature memory");
    MMDestroyFeature(&hMMFeature);
    MMCPLDebug("MiraMon", "MiraMon temporary feature memory");
    memset(&hMMFeature, 0, sizeof(hMMFeature));

    /* -------------------------------------------------------------------- */
    /*      Clean up.                                                       */
    /* -------------------------------------------------------------------- */

    if (poFeatureDefn)
        poFeatureDefn->Release();

    if (m_poSRS)
        m_poSRS->Release();

    if (m_fp != nullptr)
        VSIFCloseL(m_fp);

    if (papszValues != nullptr)
        CSLDestroy(papszValues);

    if (padfValues != nullptr)
        CPLFree(padfValues);
}

/****************************************************************************/
/*                            ResetReading()                                */
/****************************************************************************/

void OGRMiraMonLayer::ResetReading()

{
    if (iNextFID == 0)
        return;

    iNextFID = 0;

    //VSIFSeekL(m_fp, 0, SEEK_SET);
    if (!phMiraMonLayer)
        return;

    if (phMiraMonLayer->bIsPoint && phMiraMonLayer->MMPoint.pF)
    {
        VSIFSeekL(phMiraMonLayer->MMPoint.pF, 0, SEEK_SET);
        return;
    }
    if (phMiraMonLayer->bIsArc && !phMiraMonLayer->bIsPolygon &&
        phMiraMonLayer->MMArc.pF)
    {
        VSIFSeekL(phMiraMonLayer->MMArc.pF, 0, SEEK_SET);
        return;
    }
    if (phMiraMonLayer->bIsPolygon && phMiraMonLayer->MMPolygon.pF)
    {
        VSIFSeekL(phMiraMonLayer->MMPolygon.pF, 0, SEEK_SET);
        return;
    }
}

/****************************************************************************/
/*                         GetNextRawFeature()                              */
/****************************************************************************/

void OGRMiraMonLayer::GoToFieldOfMultipleRecord(MM_INTERNAL_FID iFID,
                                                MM_EXT_DBF_N_RECORDS nIRecord,
                                                MM_EXT_DBF_N_FIELDS nIField)

{
    fseek_function(phMiraMonLayer->pMMBDXP->pfDataBase,
                   phMiraMonLayer->pMultRecordIndex[iFID].offset +
                       (MM_FILE_OFFSET)nIRecord *
                           phMiraMonLayer->pMMBDXP->BytesPerRecord +
                       phMiraMonLayer->pMMBDXP->pField[nIField].AcumulatedBytes,
                   SEEK_SET);
}

/****************************************************************************/
/*                         GetNextRawFeature()                              */
/****************************************************************************/

OGRFeature *OGRMiraMonLayer::GetNextRawFeature()
{
    if (!phMiraMonLayer)
        return nullptr;

    if (iNextFID >= (GUInt64)phMiraMonLayer->TopHeader.nElemCount)
        return nullptr;

    OGRFeature *poFeature = GetFeature(iNextFID);

    if (!poFeature)
        return nullptr;

    iNextFID++;
    return poFeature;
}

/****************************************************************************/
/*                         GetFeature()                                     */
/****************************************************************************/

OGRFeature *OGRMiraMonLayer::GetFeature(GIntBig nFeatureId)

{
    OGRGeometry *poGeom = nullptr;
    OGRPoint *poPoint = nullptr;
    OGRLineString *poLS = nullptr;
    MM_INTERNAL_FID nIElem = (MM_INTERNAL_FID)nFeatureId;
    MM_EXT_DBF_N_MULTIPLE_RECORDS nIRecord = 0;

    if (!phMiraMonLayer)
        return nullptr;

    if (nFeatureId < 0)
        return nullptr;

    if (nIElem > phMiraMonLayer->TopHeader.nElemCount)
        return nullptr;

    /* -------------------------------------------------------------------- */
    /*      Read nFeatureId feature directly from the file.                 */
    /* -------------------------------------------------------------------- */
    switch (phMiraMonLayer->eLT)
    {
        case MM_LayerType_Point:
        case MM_LayerType_Point3d:
            // Read point
            poGeom = new OGRPoint();
            poPoint = poGeom->toPoint();

            // Get X,Y (z). MiraMon has no multipoints
            if (MMGetGeoFeatureFromVector(phMiraMonLayer, nIElem))
            {
                delete poGeom;
                return nullptr;
            }

            poPoint->setX(phMiraMonLayer->ReadFeature.pCoord[0].dfX);
            poPoint->setY(phMiraMonLayer->ReadFeature.pCoord[0].dfY);
            if (phMiraMonLayer->TopHeader.bIs3d)
                poPoint->setZ(phMiraMonLayer->ReadFeature.pZCoord[0]);
            break;

        case MM_LayerType_Arc:
        case MM_LayerType_Arc3d:
            poGeom = new OGRLineString();
            poLS = poGeom->toLineString();

            // Get X,Y (Z) n times MiraMon has no multilines
            if (MMGetGeoFeatureFromVector(phMiraMonLayer, nIElem))
            {
                delete poGeom;
                return nullptr;
            }

            for (MM_N_VERTICES_TYPE nIVrt = 0;
                 nIVrt < phMiraMonLayer->ReadFeature.pNCoordRing[0]; nIVrt++)
            {
                if (phMiraMonLayer->TopHeader.bIs3d)
                    poLS->addPoint(
                        phMiraMonLayer->ReadFeature.pCoord[nIVrt].dfX,
                        phMiraMonLayer->ReadFeature.pCoord[nIVrt].dfY,
                        phMiraMonLayer->ReadFeature.pZCoord[nIVrt]);
                else
                    poLS->addPoint(
                        phMiraMonLayer->ReadFeature.pCoord[nIVrt].dfX,
                        phMiraMonLayer->ReadFeature.pCoord[nIVrt].dfY);
            }
            break;

        case MM_LayerType_Pol:
        case MM_LayerType_Pol3d:
            // Read polygon
            OGRPolygon poPoly;
            MM_POLYGON_RINGS_COUNT nIRing;
            MM_N_VERTICES_TYPE nIVrtAcum;

            if (phMiraMonLayer->TopHeader.bIsMultipolygon)
            {
                OGRMultiPolygon *poMP = nullptr;

                poGeom = new OGRMultiPolygon();
                poMP = poGeom->toMultiPolygon();

                // Get X,Y (Z) n times MiraMon has no multilines
                if (MMGetGeoFeatureFromVector(phMiraMonLayer, nIElem))
                {
                    delete poGeom;
                    return nullptr;
                }

                nIVrtAcum = 0;
                if (!(phMiraMonLayer->ReadFeature.flag_VFG[0] |
                      MM_EXTERIOR_ARC_SIDE))
                {
                    CPLError(CE_Failure, CPLE_NoWriteAccess,
                             "\nWrong polygon format.");
                    delete poGeom;
                    return nullptr;
                }
                MM_BOOLEAN IAmExternal;

                for (nIRing = 0; nIRing < phMiraMonLayer->ReadFeature.nNRings;
                     nIRing++)
                {
                    OGRLinearRing poRing;

                    IAmExternal = (MM_BOOLEAN)(phMiraMonLayer->ReadFeature
                                                   .flag_VFG[nIRing] |
                                               MM_EXTERIOR_ARC_SIDE);

                    for (MM_N_VERTICES_TYPE nIVrt = 0;
                         nIVrt <
                         phMiraMonLayer->ReadFeature.pNCoordRing[nIRing];
                         nIVrt++)
                    {
                        if (phMiraMonLayer->TopHeader.bIs3d)
                        {
                            poRing.addPoint(
                                phMiraMonLayer->ReadFeature.pCoord[nIVrtAcum]
                                    .dfX,
                                phMiraMonLayer->ReadFeature.pCoord[nIVrtAcum]
                                    .dfY,
                                phMiraMonLayer->ReadFeature.pZCoord[nIVrtAcum]);
                        }
                        else
                        {
                            poRing.addPoint(
                                phMiraMonLayer->ReadFeature.pCoord[nIVrtAcum]
                                    .dfX,
                                phMiraMonLayer->ReadFeature.pCoord[nIVrtAcum]
                                    .dfY);
                        }

                        nIVrtAcum++;
                    }

                    // If I'm going to start a new polygon...
                    if ((IAmExternal &&
                         nIRing + 1 < phMiraMonLayer->ReadFeature.nNRings &&
                         ((phMiraMonLayer->ReadFeature.flag_VFG[nIRing + 1]) &
                          MM_EXTERIOR_ARC_SIDE)) ||
                        nIRing + 1 >= phMiraMonLayer->ReadFeature.nNRings)
                    {
                        poPoly.addRing(&poRing);
                        poMP->addGeometry(&poPoly);
                        poPoly.empty();
                    }
                    else
                        poPoly.addRing(&poRing);
                }
            }
            else
            {
                OGRPolygon *poP = nullptr;

                poGeom = new OGRPolygon();
                poP = poGeom->toPolygon();

                // Get X,Y (Z) n times because MiraMon has no multilinetrings
                if (MMGetGeoFeatureFromVector(phMiraMonLayer, nIElem))
                {
                    delete poGeom;
                    return nullptr;
                }

                if (phMiraMonLayer->ReadFeature.nNRings &&
                    phMiraMonLayer->ReadFeature.nNumpCoord)
                {
                    nIVrtAcum = 0;
                    if (!(phMiraMonLayer->ReadFeature.flag_VFG[0] |
                          MM_EXTERIOR_ARC_SIDE))
                    {
                        CPLError(CE_Failure, CPLE_AssertionFailed,
                                 "\nWrong polygon format.");
                        delete poGeom;
                        return nullptr;
                    }

                    for (nIRing = 0;
                         nIRing < phMiraMonLayer->ReadFeature.nNRings; nIRing++)
                    {
                        OGRLinearRing poRing;

                        for (MM_N_VERTICES_TYPE nIVrt = 0;
                             nIVrt <
                             phMiraMonLayer->ReadFeature.pNCoordRing[nIRing];
                             nIVrt++)
                        {
                            if (phMiraMonLayer->TopHeader.bIs3d)
                            {
                                poRing.addPoint(phMiraMonLayer->ReadFeature
                                                    .pCoord[nIVrtAcum]
                                                    .dfX,
                                                phMiraMonLayer->ReadFeature
                                                    .pCoord[nIVrtAcum]
                                                    .dfY,
                                                phMiraMonLayer->ReadFeature
                                                    .pZCoord[nIVrtAcum]);
                            }
                            else
                            {
                                poRing.addPoint(phMiraMonLayer->ReadFeature
                                                    .pCoord[nIVrtAcum]
                                                    .dfX,
                                                phMiraMonLayer->ReadFeature
                                                    .pCoord[nIVrtAcum]
                                                    .dfY);
                            }

                            nIVrtAcum++;
                        }
                        poP->addRing(&poRing);
                    }
                }
            }

            break;
    }

    if (poGeom == nullptr)
        return nullptr;

    /* -------------------------------------------------------------------- */
    /*      Create feature.                                                 */
    /* -------------------------------------------------------------------- */
    OGRFeature *poFeature = new OGRFeature(poFeatureDefn);
    poGeom->assignSpatialReference(m_poSRS);
    poFeature->SetGeometryDirectly(poGeom);
    poFeature->SetFID(nFeatureId);

    /* -------------------------------------------------------------------- */
    /*      Process field values.                                           */
    /* -------------------------------------------------------------------- */
    if (phMiraMonLayer->pMMBDXP)
    {
        MM_EXT_DBF_N_FIELDS nIField;

        for (nIField = 0; nIField < phMiraMonLayer->pMMBDXP->nFields; nIField++)
        {
            MMResizeStringToOperateIfNeeded(
                phMiraMonLayer,
                phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField);

            if (poFeature->GetDefnRef()->GetFieldDefn(nIField)->GetType() ==
                    OFTStringList ||
                (poFeature->GetDefnRef()->GetFieldDefn(nIField)->GetType() ==
                     OFTString &&
                 poFeature->GetDefnRef()->GetFieldDefn(nIField)->GetSubType() ==
                     OFSTJSON))
            {
                if (poFeature->GetDefnRef()
                        ->GetFieldDefn(nIField)
                        ->GetSubType() == OFSTJSON)
                {
                    // REVISAR
                    MMResizeStringToOperateIfNeeded(
                        phMiraMonLayer,
                        phMiraMonLayer->pMMBDXP->BytesPerRecord +
                            2 * phMiraMonLayer->pMultRecordIndex[nIElem].nMR +
                            8);
                    strcpy(phMiraMonLayer->szStringToOperate, "(``[");
                    size_t nBytes = 4;
                    for (nIRecord = 0;
                         nIRecord <
                         phMiraMonLayer->pMultRecordIndex[nIElem].nMR;
                         nIRecord++)
                    {
                        GoToFieldOfMultipleRecord(nIElem, nIRecord, nIField);

                        fread_function(phMiraMonLayer->szStringToOperate +
                                           nBytes,
                                       phMiraMonLayer->pMMBDXP->pField[nIField]
                                           .BytesPerField,
                                       1, phMiraMonLayer->pMMBDXP->pfDataBase);
                        (phMiraMonLayer->szStringToOperate +
                         nBytes)[phMiraMonLayer->pMMBDXP->pField[nIField]
                                     .BytesPerField] = '\0';
                        MM_RemoveLeadingWhitespaceOfString(
                            phMiraMonLayer->szStringToOperate + nBytes);
                        MM_RemoveWhitespacesFromEndOfString(
                            phMiraMonLayer->szStringToOperate);

                        nBytes +=
                            strlen(phMiraMonLayer->szStringToOperate + nBytes);
                        if (phMiraMonLayer->pMMBDXP->CharSet ==
                            MM_JOC_CARAC_OEM850_DBASE)
                            MM_oemansi_n(
                                phMiraMonLayer->szStringToOperate + nBytes,
                                phMiraMonLayer->pMMBDXP->pField[nIField]
                                    .BytesPerField);

                        if (phMiraMonLayer->pMMBDXP->CharSet !=
                            MM_JOC_CARAC_UTF8_DBF)
                        {
                            // MiraMon encoding is ISO 8859-1 (Latin1) -> Recode to UTF-8
                            char *pszString = CPLRecode(
                                phMiraMonLayer->szStringToOperate + nBytes,
                                CPL_ENC_ISO8859_1, CPL_ENC_UTF8);

                            CPLStrlcpy(
                                phMiraMonLayer->szStringToOperate + nBytes,
                                pszString,
                                (size_t)phMiraMonLayer->pMMBDXP->pField[nIField]
                                        .BytesPerField +
                                    1);

                            CPLFree(pszString);
                        }

                        if (nIRecord <
                            phMiraMonLayer->pMultRecordIndex[nIElem].nMR - 1)
                        {
                            strcat(phMiraMonLayer->szStringToOperate + nBytes,
                                   ",");
                            nBytes += 1;
                        }
                        else
                        {
                            strcat(phMiraMonLayer->szStringToOperate + nBytes,
                                   "]``)");
                            nBytes += 4;
                        }
                    }
                    poFeature->SetField(nIField,
                                        phMiraMonLayer->szStringToOperate);
                }
                else
                {
                    for (nIRecord = 0;
                         nIRecord <
                         phMiraMonLayer->pMultRecordIndex[nIElem].nMR;
                         nIRecord++)
                    {
                        GoToFieldOfMultipleRecord(nIElem, nIRecord, nIField);
                        memset(phMiraMonLayer->szStringToOperate, 0,
                               phMiraMonLayer->pMMBDXP->pField[nIField]
                                   .BytesPerField);
                        fread_function(phMiraMonLayer->szStringToOperate,
                                       phMiraMonLayer->pMMBDXP->pField[nIField]
                                           .BytesPerField,
                                       1, phMiraMonLayer->pMMBDXP->pfDataBase);
                        phMiraMonLayer
                            ->szStringToOperate[phMiraMonLayer->pMMBDXP
                                                    ->pField[nIField]
                                                    .BytesPerField] = '\0';
                        MM_RemoveWhitespacesFromEndOfString(
                            phMiraMonLayer->szStringToOperate);

                        if (phMiraMonLayer->pMMBDXP->CharSet ==
                            MM_JOC_CARAC_OEM850_DBASE)
                            MM_oemansi_n(
                                phMiraMonLayer->szStringToOperate,
                                phMiraMonLayer->pMMBDXP->pField[nIField]
                                    .BytesPerField);

                        if (phMiraMonLayer->pMMBDXP->CharSet !=
                            MM_JOC_CARAC_UTF8_DBF)
                        {
                            // MiraMon encoding is ISO 8859-1 (Latin1) -> Recode to UTF-8
                            char *pszString =
                                CPLRecode(phMiraMonLayer->szStringToOperate,
                                          CPL_ENC_ISO8859_1, CPL_ENC_UTF8);

                            CPLStrlcpy(
                                phMiraMonLayer->szStringToOperate, pszString,
                                (size_t)phMiraMonLayer->pMMBDXP->pField[nIField]
                                        .BytesPerField +
                                    1);

                            CPLFree(pszString);
                        }
                        papszValues[nIRecord] =
                            CPLStrdup(phMiraMonLayer->szStringToOperate);
                    }
                    papszValues[nIRecord] =
                        nullptr;  // Necessary to finish the list
                    poFeature->SetField(nIField, papszValues);
                }
            }
            else if (poFeature->GetDefnRef()
                         ->GetFieldDefn(nIField)
                         ->GetType() == OFTString)
            {
                if (!phMiraMonLayer->pMultRecordIndex ||
                    phMiraMonLayer->pMultRecordIndex[nIElem].nMR == 0)
                {
                    memset(
                        phMiraMonLayer->szStringToOperate, 0,
                        phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField);
                    continue;
                }
                if (phMiraMonLayer->iMultiRecord != -2)
                {
                    if (phMiraMonLayer->iMultiRecord == -1)
                        GoToFieldOfMultipleRecord(
                            nIElem,
                            phMiraMonLayer->pMultRecordIndex[nIElem].nMR - 1,
                            nIField);
                    else if ((MM_EXT_DBF_N_MULTIPLE_RECORDS)
                                 phMiraMonLayer->iMultiRecord <
                             phMiraMonLayer->pMultRecordIndex[nIElem].nMR)
                        GoToFieldOfMultipleRecord(
                            nIElem,
                            (MM_EXT_DBF_N_MULTIPLE_RECORDS)
                                phMiraMonLayer->iMultiRecord,
                            nIField);
                    else
                    {
                        memset(phMiraMonLayer->szStringToOperate, 0,
                               phMiraMonLayer->pMMBDXP->pField[nIField]
                                   .BytesPerField);
                        continue;
                    }
                }
                else
                    GoToFieldOfMultipleRecord(nIElem, 0, nIField);

                memset(phMiraMonLayer->szStringToOperate, 0,
                       phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField);
                fread_function(
                    phMiraMonLayer->szStringToOperate,
                    phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField, 1,
                    phMiraMonLayer->pMMBDXP->pfDataBase);
                phMiraMonLayer
                    ->szStringToOperate[phMiraMonLayer->pMMBDXP->pField[nIField]
                                            .BytesPerField] = '\0';
                MM_RemoveWhitespacesFromEndOfString(
                    phMiraMonLayer->szStringToOperate);

                if (phMiraMonLayer->pMMBDXP->CharSet ==
                    MM_JOC_CARAC_OEM850_DBASE)
                    MM_oemansi(phMiraMonLayer->szStringToOperate);

                if (phMiraMonLayer->pMMBDXP->CharSet != MM_JOC_CARAC_UTF8_DBF)
                {
                    // MiraMon encoding is ISO 8859-1 (Latin1) -> Recode to UTF-8
                    char *pszString =
                        CPLRecode(phMiraMonLayer->szStringToOperate,
                                  CPL_ENC_ISO8859_1, CPL_ENC_UTF8);
                    CPLStrlcpy(phMiraMonLayer->szStringToOperate, pszString,
                               (size_t)phMiraMonLayer->pMMBDXP->pField[nIField]
                                       .BytesPerField +
                                   1);
                    CPLFree(pszString);
                }
                poFeature->SetField(nIField, phMiraMonLayer->szStringToOperate);
            }
            else if (poFeature->GetDefnRef()
                             ->GetFieldDefn(nIField)
                             ->GetType() == OFTIntegerList ||
                     poFeature->GetDefnRef()
                             ->GetFieldDefn(nIField)
                             ->GetType() == OFTInteger64List ||
                     poFeature->GetDefnRef()
                             ->GetFieldDefn(nIField)
                             ->GetType() == OFTRealList)
            {
                for (nIRecord = 0;
                     nIRecord < phMiraMonLayer->pMultRecordIndex[nIElem].nMR;
                     nIRecord++)
                {
                    GoToFieldOfMultipleRecord(nIElem, nIRecord, nIField);
                    memset(
                        phMiraMonLayer->szStringToOperate, 0,
                        phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField);
                    fread_function(
                        phMiraMonLayer->szStringToOperate,
                        phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField,
                        1, phMiraMonLayer->pMMBDXP->pfDataBase);
                    phMiraMonLayer->szStringToOperate[phMiraMonLayer->pMMBDXP
                                                          ->pField[nIField]
                                                          .BytesPerField] =
                        '\0';

                    padfValues[nIRecord] =
                        atof(phMiraMonLayer->szStringToOperate);
                }

                poFeature->SetField(
                    nIField, phMiraMonLayer->pMultRecordIndex[nIElem].nMR,
                    padfValues);
            }
            else if (poFeature->GetDefnRef()
                             ->GetFieldDefn(nIField)
                             ->GetType() == OFTInteger ||
                     poFeature->GetDefnRef()
                             ->GetFieldDefn(nIField)
                             ->GetType() == OFTInteger64 ||
                     poFeature->GetDefnRef()
                             ->GetFieldDefn(nIField)
                             ->GetType() == OFTReal)
            {
                if (!phMiraMonLayer->pMultRecordIndex ||
                    phMiraMonLayer->pMultRecordIndex[nIElem].nMR == 0)
                {
                    memset(
                        phMiraMonLayer->szStringToOperate, 0,
                        phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField);
                    continue;
                }
                if (phMiraMonLayer->iMultiRecord != -2)
                {
                    if (phMiraMonLayer->iMultiRecord == -1)
                        GoToFieldOfMultipleRecord(
                            nIElem,
                            phMiraMonLayer->pMultRecordIndex[nIElem].nMR - 1,
                            nIField);
                    else if ((MM_EXT_DBF_N_MULTIPLE_RECORDS)
                                 phMiraMonLayer->iMultiRecord <
                             phMiraMonLayer->pMultRecordIndex[nIElem].nMR)
                        GoToFieldOfMultipleRecord(
                            nIElem,
                            (MM_EXT_DBF_N_MULTIPLE_RECORDS)
                                phMiraMonLayer->iMultiRecord,
                            nIField);
                    else
                    {
                        memset(phMiraMonLayer->szStringToOperate, 0,
                               phMiraMonLayer->pMMBDXP->pField[nIField]
                                   .BytesPerField);
                        continue;
                    }
                }
                else
                    GoToFieldOfMultipleRecord(nIElem, 0, nIField);

                memset(phMiraMonLayer->szStringToOperate, 0,
                       phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField);
                fread_function(
                    phMiraMonLayer->szStringToOperate,
                    phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField, 1,
                    phMiraMonLayer->pMMBDXP->pfDataBase);
                phMiraMonLayer
                    ->szStringToOperate[phMiraMonLayer->pMMBDXP->pField[nIField]
                                            .BytesPerField] = '\0';
                MM_RemoveWhitespacesFromEndOfString(
                    phMiraMonLayer->szStringToOperate);
                poFeature->SetField(nIField,
                                    atof(phMiraMonLayer->szStringToOperate));
            }
            else if (poFeature->GetDefnRef()
                         ->GetFieldDefn(nIField)
                         ->GetType() == OFTDate)
            {
                if (phMiraMonLayer->pMultRecordIndex[nIElem].nMR == 0)
                {
                    memset(
                        phMiraMonLayer->szStringToOperate, 0,
                        phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField);
                    continue;
                }
                if (phMiraMonLayer->iMultiRecord != -2)
                {
                    if (phMiraMonLayer->iMultiRecord == -1)
                        GoToFieldOfMultipleRecord(
                            nIElem,
                            phMiraMonLayer->pMultRecordIndex[nIElem].nMR - 1,
                            nIField);
                    else if ((MM_EXT_DBF_N_MULTIPLE_RECORDS)
                                 phMiraMonLayer->iMultiRecord <
                             phMiraMonLayer->pMultRecordIndex[nIElem].nMR)
                        GoToFieldOfMultipleRecord(
                            nIElem,
                            (MM_EXT_DBF_N_MULTIPLE_RECORDS)
                                phMiraMonLayer->iMultiRecord,
                            nIField);
                    else
                    {
                        memset(phMiraMonLayer->szStringToOperate, 0,
                               phMiraMonLayer->pMMBDXP->pField[nIField]
                                   .BytesPerField);
                        continue;
                    }
                }
                else
                    GoToFieldOfMultipleRecord(nIElem, 0, nIField);

                memset(phMiraMonLayer->szStringToOperate, 0,
                       phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField);
                fread_function(
                    phMiraMonLayer->szStringToOperate,
                    phMiraMonLayer->pMMBDXP->pField[nIField].BytesPerField, 1,
                    phMiraMonLayer->pMMBDXP->pfDataBase);
                phMiraMonLayer
                    ->szStringToOperate[phMiraMonLayer->pMMBDXP->pField[nIField]
                                            .BytesPerField] = '\0';

                MM_RemoveWhitespacesFromEndOfString(
                    phMiraMonLayer->szStringToOperate);
                if (!MMIsEmptyString(phMiraMonLayer->szStringToOperate))
                {
                    char pszDate_5[5];
                    char pszDate_3[3];
                    int Year, Month, Day;

                    strncpy(pszDate_5, phMiraMonLayer->szStringToOperate, 4);
                    pszDate_5[4] = '\0';
                    Year = atoi(pszDate_5);

                    strncpy(pszDate_3, phMiraMonLayer->szStringToOperate + 4,
                            2);
                    (pszDate_3)[2] = '\0';
                    Month = atoi(pszDate_3);

                    strncpy(pszDate_3, phMiraMonLayer->szStringToOperate + 6,
                            2);
                    (pszDate_3)[2] = '\0';
                    Day = atoi(pszDate_3);

                    poFeature->SetField(nIField, Year, Month, Day);
                }
                else
                    poFeature->SetField(nIField,
                                        phMiraMonLayer->szStringToOperate);
            }
        }
    }

    m_nFeaturesRead++;

    return poFeature;
}

/****************************************************************************/
/*                         GetFeatureCount()                                */
/****************************************************************************/
GIntBig OGRMiraMonLayer::GetFeatureCount(int bForce)
{
    if (!phMiraMonLayer || m_poFilterGeom != nullptr ||
        m_poAttrQuery != nullptr)
        return OGRLayer::GetFeatureCount(bForce);

    return (GIntBig)phMiraMonLayer->TopHeader.nElemCount;
}

/****************************************************************************/
/*                      MMProcessMultiGeometry()                            */
/****************************************************************************/
OGRErr OGRMiraMonLayer::MMProcessMultiGeometry(OGRGeometryH hGeom,
                                               OGRFeature *poFeature)

{
    OGRErr eErr = OGRERR_NONE;
    OGRGeometry *poGeom = LOG_ACTION(OGRGeometry::FromHandle(hGeom));

    if (poGeom == nullptr)
    {
        CPLError(
            CE_Failure, CPLE_AppDefined,
            "\nFeatures without geometry not supported by MiraMon writer.");
        return LOG_ACTION(OGRERR_FAILURE);
    }

    if (poGeom->getGeometryType() == wkbUnknown)
        return LOG_ACTION(OGRERR_UNSUPPORTED_GEOMETRY_TYPE);

    // MUltigeometry field processing (just in case of a MG inside a MG)
    if (wkbFlatten(poGeom->getGeometryType()) == wkbGeometryCollection)
    {
        int nGeom = OGR_G_GetGeometryCount(OGRGeometry::ToHandle(poGeom));
        for (int iGeom = 0; iGeom < nGeom; iGeom++)
        {
            OGRGeometryH poNewGeometry =
                OGR_G_GetGeometryRef(OGRGeometry::ToHandle(poGeom), iGeom);
            eErr = MMProcessMultiGeometry(poNewGeometry, poFeature);
            if (eErr != OGRERR_NONE)
                return eErr;
        }
        return eErr;
    }
    // Converting multilines and multi points to simple ones
    if (wkbFlatten(poGeom->getGeometryType()) == wkbMultiLineString ||
        wkbFlatten(poGeom->getGeometryType()) == wkbMultiPoint)
    {
        int nGeom = OGR_G_GetGeometryCount(OGRGeometry::ToHandle(poGeom));
        for (int iGeom = 0; iGeom < nGeom; iGeom++)
        {
            OGRGeometryH poNewGeometry =
                OGR_G_GetGeometryRef(OGRGeometry::ToHandle(poGeom), iGeom);
            eErr = MMProcessGeometry(poNewGeometry, poFeature, (iGeom == 0));
            if (eErr != OGRERR_NONE)
                return eErr;
        }
        return eErr;
    }

    // Processing a simple geometry
    return LOG_ACTION(
        MMProcessGeometry(OGRGeometry::ToHandle(poGeom), poFeature, TRUE));
}

/****************************************************************************/
/*                           MMProcessGeometry()                            */
/****************************************************************************/
OGRErr OGRMiraMonLayer::MMProcessGeometry(OGRGeometryH hGeom,
                                          OGRFeature *poFeature,
                                          MM_BOOLEAN bcalculateRecord)

{
    OGRErr eErr = OGRERR_NONE;
    OGRGeometry *poGeom = nullptr;
    if (hGeom)
    {
        poGeom = OGRGeometry::FromHandle(hGeom);

        // Translating types from GDAL to MiraMon
        int eLT = LOG_ACTION(poGeom->getGeometryType());
        switch (wkbFlatten(eLT))
        {
            case wkbPoint:
                phMiraMonLayer = LOG_ACTION(&hMiraMonLayerPNT);
                if (OGR_G_Is3D(hGeom))
                    phMiraMonLayer->eLT = MM_LayerType_Point3d;
                else
                    phMiraMonLayer->eLT = MM_LayerType_Point;
                break;
            case wkbLineString:
                phMiraMonLayer = LOG_ACTION(&hMiraMonLayerARC);
                if (OGR_G_Is3D(hGeom))
                    phMiraMonLayer->eLT = MM_LayerType_Arc3d;
                else
                    phMiraMonLayer->eLT = MM_LayerType_Arc;
                break;
            case wkbPolygon:
            case wkbMultiPolygon:
            case wkbPolyhedralSurface:
            case wkbTIN:
            case wkbTriangle:
                phMiraMonLayer = LOG_ACTION(&hMiraMonLayerPOL);
                if (OGR_G_Is3D(hGeom))
                    phMiraMonLayer->eLT = MM_LayerType_Pol3d;
                else
                    phMiraMonLayer->eLT = MM_LayerType_Pol;
                break;
            case wkbUnknown:
            default:
            {
                CPLError(CE_Warning, CPLE_NotSupported,
                         "MiraMon "
                         "does not support geometry type '%d'",
                         eLT);
                return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
            }
        }
    }
    else
    {
        // Processing only the table. A DBF will be generated
        phMiraMonLayer = LOG_ACTION(&hMiraMonLayerReadOrNonGeom);
        phMiraMonLayer->eLT = MM_LayerType_Unknown;
    }

    /* -------------------------------------------------------------------- */
    /*      Field translation from GDAL to MiraMon                          */
    /* -------------------------------------------------------------------- */
    // Reset the object where readed coordinates are going to be stored
    MMResetFeatureGeometry(&hMMFeature);
    if (bcalculateRecord)
    {
        MMResetFeatureRecord(&hMMFeature);
        if (!phMiraMonLayer->pLayerDB)
        {
            eErr = TranslateFieldsToMM();
            if (eErr != OGRERR_NONE)
                return eErr;
        }
        // Content field translation from GDAL to MiraMon
        eErr = TranslateFieldsValuesToMM(poFeature);
        if (eErr != OGRERR_NONE)
        {
            CPLDebug("MiraMon", "Error in MMProcessGeometry()");
            return eErr;
        }
    }

    /* -------------------------------------------------------------------- */
    /*      Write Geometry                                                  */
    /* -------------------------------------------------------------------- */

    // Reads objects with coordinates and transform them to MiraMon
    if (poGeom)
    {
        eErr = MMLoadGeometry(OGRGeometry::ToHandle(poGeom));
    }
    else
    {
        if (!phMiraMonLayer->bIsBeenInit)
        {
            phMiraMonLayer->bIsDBF = TRUE;
            MMInitLayerByType(phMiraMonLayer);
            phMiraMonLayer->bIsBeenInit = 1;
        }
    }

    // Writes coordinates to the disk
    if (eErr == OGRERR_NONE)
        return MMWriteGeometry();
    CPLDebug("MiraMon", "Error in MMProcessGeometry()");
    return eErr;
}

/****************************************************************************/
/*                           ICreateFeature()                               */
/****************************************************************************/

OGRErr OGRMiraMonLayer::ICreateFeature(OGRFeature *poFeature)

{
    OGRErr eErr = OGRERR_NONE;

    if (!bUpdate)
    {
        CPLError(CE_Failure, CPLE_NoWriteAccess,
                 "\nCannot create features on a read-only dataset.");
        return OGRERR_FAILURE;
    }

    /* -------------------------------------------------------------------- */
    /*      Write out the feature                                           */
    /* -------------------------------------------------------------------- */
    OGRGeometry *poGeom = LOG_ACTION(poFeature->GetGeometryRef());

    // Processing a feature without geometry.
    if (poGeom == nullptr)
    {
        eErr = LOG_ACTION(MMProcessGeometry(nullptr, poFeature, TRUE));
        if (phMiraMonLayer->bIsDBF)
            poFeature->SetFID(phMiraMonLayer->TopHeader.nElemCount - 1);
        return eErr;
    }

    // At this point MiraMon does not support unkwnon type geometry
    if (poGeom->getGeometryType() == wkbUnknown)
        return LOG_ACTION(OGRERR_UNSUPPORTED_GEOMETRY_TYPE);

    // Converting to simple geometries
    if (wkbFlatten(poGeom->getGeometryType()) == wkbGeometryCollection)
    {
        int nGeom =
            LOG_ACTION(OGR_G_GetGeometryCount(OGRGeometry::ToHandle(poGeom)));
        for (int iGeom = 0; iGeom < nGeom; iGeom++)
        {
            OGRGeometryH poNewGeometry = LOG_ACTION(
                OGR_G_GetGeometryRef(OGRGeometry::ToHandle(poGeom), iGeom));
            eErr = LOG_ACTION(MMProcessMultiGeometry(poNewGeometry, poFeature));
            if (eErr != OGRERR_NONE)
                return eErr;
        }

        return eErr;
    }

    // Processing the geometry
    eErr = LOG_ACTION(
        MMProcessMultiGeometry(OGRGeometry::ToHandle(poGeom), poFeature));

    // Set the FID from 0 index
    if (phMiraMonLayer)
    {
        if (phMiraMonLayer->bIsPolygon &&
            phMiraMonLayer->TopHeader.nElemCount > 1)
            poFeature->SetFID((GIntBig)phMiraMonLayer->TopHeader.nElemCount -
                              2);
        else if (phMiraMonLayer->TopHeader.nElemCount > 0)
            poFeature->SetFID((GIntBig)phMiraMonLayer->TopHeader.nElemCount -
                              1);
    }
    return eErr;
}

/****************************************************************************/
/*                          MMDumpVertices()                                */
/****************************************************************************/

OGRErr OGRMiraMonLayer::MMDumpVertices(OGRGeometryH hGeom,
                                       MM_BOOLEAN bExternalRing,
                                       MM_BOOLEAN bUseVFG)
{
    // If the MiraMonLayer structure has not been init,
    // here is the moment to do that.
    if (!phMiraMonLayer->bIsBeenInit)
    {
        if (MMInitLayerByType(phMiraMonLayer))
            return OGRERR_FAILURE;
        phMiraMonLayer->bIsBeenInit = 1;
    }
    if (MMResize_MM_N_VERTICES_TYPE_Pointer(
            &hMMFeature.pNCoordRing, &hMMFeature.nMaxpNCoordRing,
            (MM_N_VERTICES_TYPE)hMMFeature.nNRings + 1, MM_MEAN_NUMBER_OF_RINGS,
            0))
        return OGRERR_FAILURE;

    if (bUseVFG)
    {
        if (MMResizeVFGPointer(&hMMFeature.flag_VFG, &hMMFeature.nMaxVFG,
                               (MM_INTERNAL_FID)hMMFeature.nNRings + 1,
                               MM_MEAN_NUMBER_OF_RINGS, 0))
            return OGRERR_FAILURE;

        hMMFeature.flag_VFG[hMMFeature.nIRing] = MM_END_ARC_IN_RING;
        if (bExternalRing)
            hMMFeature.flag_VFG[hMMFeature.nIRing] |= MM_EXTERIOR_ARC_SIDE;
        // In MiraMon the external ring is clockwise and the internals are
        // coounterclockwise.
        OGRGeometry *poGeom = OGRGeometry::FromHandle(hGeom);
        if ((bExternalRing && !poGeom->toLinearRing()->isClockwise()) ||
            (!bExternalRing && poGeom->toLinearRing()->isClockwise()))
            hMMFeature.flag_VFG[hMMFeature.nIRing] |= MM_ROTATE_ARC;
    }

    hMMFeature.pNCoordRing[hMMFeature.nIRing] = OGR_G_GetPointCount(hGeom);

    if (MMResizeMM_POINT2DPointer(&hMMFeature.pCoord, &hMMFeature.nMaxpCoord,
                                  hMMFeature.nICoord +
                                      hMMFeature.pNCoordRing[hMMFeature.nIRing],
                                  MM_MEAN_NUMBER_OF_NCOORDS, 0))
        return OGRERR_FAILURE;
    if (MMResizeDoublePointer(&hMMFeature.pZCoord, &hMMFeature.nMaxpZCoord,
                              hMMFeature.nICoord +
                                  hMMFeature.pNCoordRing[hMMFeature.nIRing],
                              MM_MEAN_NUMBER_OF_NCOORDS, 0))
        return OGRERR_FAILURE;

    for (int iPoint = 0;
         (MM_N_VERTICES_TYPE)iPoint < hMMFeature.pNCoordRing[hMMFeature.nIRing];
         iPoint++)
    {
        hMMFeature.pCoord[hMMFeature.nICoord].dfX = OGR_G_GetX(hGeom, iPoint);
        hMMFeature.pCoord[hMMFeature.nICoord].dfY = OGR_G_GetY(hGeom, iPoint);
        if (OGR_G_GetCoordinateDimension(hGeom) == 2)
            hMMFeature.pZCoord[hMMFeature.nICoord] =
                MM_NODATA_COORD_Z;  // Possible rare case
        else
        {
            hMMFeature.pZCoord[hMMFeature.nICoord] = OGR_G_GetZ(hGeom, iPoint);
            phMiraMonLayer->bIsReal3d = 1;
        }

        hMMFeature.nICoord++;
    }
    hMMFeature.nIRing++;
    hMMFeature.nNRings++;
    return OGRERR_NONE;
}

/****************************************************************************/
/*                           MMLoadGeometry()                               */
/*                                                                          */
/*      Loads on a MiraMon object Feature all coordinates from feature      */
/*                                                                          */
/****************************************************************************/
OGRErr OGRMiraMonLayer::MMLoadGeometry(OGRGeometryH hGeom)

{
    OGRErr eErr = OGRERR_NONE;
    MM_BOOLEAN bExternalRing;

    /* -------------------------------------------------------------------- */
    /*      This is a geometry with sub-geometries.                         */
    /* -------------------------------------------------------------------- */
    int nGeom = OGR_G_GetGeometryCount(hGeom);

    int eLT = LOG_ACTION(wkbFlatten(OGR_G_GetGeometryType(hGeom)));

    if (eLT == wkbMultiPolygon || eLT == wkbPolyhedralSurface ||
        eLT == wkbTIN || eLT == wkbTriangle)
    {
        for (int iGeom = 0; iGeom < nGeom; iGeom++)
        {
            OGRGeometryH poNewGeometry = OGR_G_GetGeometryRef(hGeom, iGeom);

            // Reads all coordinates
            eErr = MMLoadGeometry(poNewGeometry);
            if (eErr != OGRERR_NONE)
                return eErr;
        }
    }
    else if (eLT == wkbPolygon)
    {
        for (int iGeom = 0; iGeom < nGeom && eErr == OGRERR_NONE; iGeom++)
        {
            OGRGeometryH poNewGeometry = OGR_G_GetGeometryRef(hGeom, iGeom);

            if (iGeom == 0)
                bExternalRing = true;
            else
                bExternalRing = false;

            eErr = MMDumpVertices(poNewGeometry, bExternalRing, TRUE);
            if (eErr != OGRERR_NONE)
                return eErr;
        }
    }
    else if (eLT == wkbPoint || eLT == wkbLineString)
    {
        // Reads all coordinates
        eErr = MMDumpVertices(hGeom, true, FALSE);

        if (eErr != OGRERR_NONE)
            return eErr;
    }
    else if (eLT == wkbGeometryCollection)
    {
        CPLError(
            CE_Failure, CPLE_NotSupported,
            "MiraMon: wkbGeometryCollection inside a wkbGeometryCollection?");
        return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
    }

    return OGRERR_NONE;
}

/****************************************************************************/
/*                           WriteGeometry()                                */
/*                                                                          */
/*      Write a geometry to the file.  If bExternalRing is true it          */
/*      means the ring is being processed is external.                      */
/*                                                                          */
/****************************************************************************/

OGRErr OGRMiraMonLayer::MMWriteGeometry()

{
    OGRErr eErr = MMAddFeature(phMiraMonLayer, &hMMFeature);

    if (eErr == MM_FATAL_ERROR_WRITING_FEATURES)
    {
        CPLDebug("MiraMon", "Error in MMAddFeature() "
                            "MM_FATAL_ERROR_WRITING_FEATURES");
        CPLError(CE_Failure, CPLE_FileIO, "\nMiraMon write failure: %s",
                 VSIStrerror(errno));
        return OGRERR_FAILURE;
    }
    if (eErr == MM_STOP_WRITING_FEATURES)
    {
        CPLDebug("MiraMon", "Error in MMAddFeature() "
                            "MM_STOP_WRITING_FEATURES");
        CPLError(CE_Failure, CPLE_FileIO, "MiraMon format limitations.");
        CPLError(CE_Failure, CPLE_FileIO,
                 "Try V2.0 option (-lco Version=V2.0).");
        return OGRERR_FAILURE;
    }

    return OGRERR_NONE;
}

/****************************************************************************/
/*                       TranslateFieldsToMM()                              */
/*                                                                          */
/*      Translase ogr Fields to a structure that MiraMon can understand     */
/****************************************************************************/

OGRErr OGRMiraMonLayer::TranslateFieldsToMM()

{
    if (poFeatureDefn->GetFieldCount() == 0)
        return OGRERR_NONE;

    CPLDebug("MiraMon", "Translating fields to MiraMon...");
    // If the structure is filled we do anything
    if (phMiraMonLayer->pLayerDB)
        return OGRERR_NONE;

    phMiraMonLayer->pLayerDB = static_cast<struct MiraMonDataBase *>(
        CPLCalloc(sizeof(*phMiraMonLayer->pLayerDB), 1));
    if (!phMiraMonLayer->pLayerDB)
        return OGRERR_NOT_ENOUGH_MEMORY;

    phMiraMonLayer->pLayerDB->pFields =
        static_cast<struct MiraMonDataBaseField *>(
            CPLCalloc(poFeatureDefn->GetFieldCount(),
                      sizeof(*(phMiraMonLayer->pLayerDB->pFields))));
    if (!phMiraMonLayer->pLayerDB->pFields)
        return OGRERR_NOT_ENOUGH_MEMORY;

    phMiraMonLayer->pLayerDB->nNFields = 0;
    if (phMiraMonLayer->pLayerDB->pFields)
    {
        memset(phMiraMonLayer->pLayerDB->pFields, 0,
               poFeatureDefn->GetFieldCount() *
                   sizeof(*phMiraMonLayer->pLayerDB->pFields));
        for (MM_EXT_DBF_N_FIELDS iField = 0;
             iField < (MM_EXT_DBF_N_FIELDS)poFeatureDefn->GetFieldCount();
             iField++)
        {
            switch (poFeatureDefn->GetFieldDefn(iField)->GetType())
            {
                case OFTInteger:
                case OFTIntegerList:
                    phMiraMonLayer->pLayerDB->pFields[iField].eFieldType =
                        MM_Numeric;
                    phMiraMonLayer->pLayerDB->pFields[iField]
                        .nNumberOfDecimals = 0;
                    break;

                case OFTInteger64:
                case OFTInteger64List:
                    phMiraMonLayer->pLayerDB->pFields[iField].bIs64BitInteger =
                        TRUE;
                    phMiraMonLayer->pLayerDB->pFields[iField].eFieldType =
                        MM_Numeric;
                    phMiraMonLayer->pLayerDB->pFields[iField]
                        .nNumberOfDecimals = 0;
                    break;

                case OFTReal:
                case OFTRealList:
                    phMiraMonLayer->pLayerDB->pFields[iField].eFieldType =
                        MM_Numeric;
                    phMiraMonLayer->pLayerDB->pFields[iField]
                        .nNumberOfDecimals =
                        poFeatureDefn->GetFieldDefn(iField)->GetPrecision();
                    break;

                case OFTBinary:
                    phMiraMonLayer->pLayerDB->pFields[iField].eFieldType =
                        MM_Logic;
                    break;

                case OFTDate:
                    phMiraMonLayer->pLayerDB->pFields[iField].eFieldType =
                        MM_Data;
                    break;

                case OFTTime:
                case OFTDateTime:
                    phMiraMonLayer->pLayerDB->pFields[iField].eFieldType =
                        MM_Character;
                    break;

                case OFTString:
                case OFTStringList:
                default:
                    phMiraMonLayer->pLayerDB->pFields[iField].eFieldType =
                        MM_Character;
                    break;
            }
            if (poFeatureDefn->GetFieldDefn(iField)->GetType() == OFTDate)
                phMiraMonLayer->pLayerDB->pFields[iField].nFieldSize = 8;
            else
            {
                if (poFeatureDefn->GetFieldDefn(iField)->GetPrecision() == 0)
                {
                    phMiraMonLayer->pLayerDB->pFields[iField].nFieldSize =
                        poFeatureDefn->GetFieldDefn(iField)->GetWidth();
                    if (phMiraMonLayer->pLayerDB->pFields[iField].nFieldSize ==
                        0)
                        phMiraMonLayer->pLayerDB->pFields[iField].nFieldSize =
                            1;
                }
                else
                {
                    // One more space for the "."
                    phMiraMonLayer->pLayerDB->pFields[iField].nFieldSize =
                        (unsigned int)(poFeatureDefn->GetFieldDefn(iField)
                                           ->GetWidth() +
                                       1);
                }
            }

            if (poFeatureDefn->GetFieldDefn(iField)->GetNameRef())
            {
                // Interlis 1 encoding is ISO 8859-1 (Latin1) -> Recode from UTF-8
                if (phMiraMonLayer->nCharSet != MM_JOC_CARAC_UTF8_DBF)
                {
                    char *pszString = CPLRecode(
                        poFeatureDefn->GetFieldDefn(iField)->GetNameRef(),
                        CPL_ENC_UTF8, CPL_ENC_ISO8859_1);
                    MM_strnzcpy(
                        phMiraMonLayer->pLayerDB->pFields[iField].pszFieldName,
                        pszString, MM_MAX_LON_FIELD_NAME_DBF);
                    CPLFree(pszString);
                }
                else
                {
                    MM_strnzcpy(
                        phMiraMonLayer->pLayerDB->pFields[iField].pszFieldName,
                        poFeatureDefn->GetFieldDefn(iField)->GetNameRef(),
                        MM_MAX_LON_FIELD_NAME_DBF);
                }
            }

            if (poFeatureDefn->GetFieldDefn(iField)->GetAlternativeNameRef())
            {
                if (phMiraMonLayer->nCharSet != MM_JOC_CARAC_UTF8_DBF)
                {
                    char *pszString =
                        CPLRecode(poFeatureDefn->GetFieldDefn(iField)
                                      ->GetAlternativeNameRef(),
                                  CPL_ENC_UTF8, CPL_ENC_ISO8859_1);
                    MM_strnzcpy(phMiraMonLayer->pLayerDB->pFields[iField]
                                    .pszFieldDescription,
                                pszString, MM_MAX_BYTES_FIELD_DESC);
                    CPLFree(pszString);
                }
                else
                {
                    MM_strnzcpy(phMiraMonLayer->pLayerDB->pFields[iField]
                                    .pszFieldDescription,
                                poFeatureDefn->GetFieldDefn(iField)
                                    ->GetAlternativeNameRef(),
                                MM_MAX_BYTES_FIELD_DESC);
                }
            }
            phMiraMonLayer->pLayerDB->nNFields++;
        }
    }

    CPLDebug("MiraMon", "Fields to MiraMon translated.");
    return OGRERR_NONE;
}

/****************************************************************************/
/*                       TranslateFieldsValuesToMM()                        */
/*                                                                          */
/*      Translase ogr Fields to a structure that MiraMon can understand     */
/****************************************************************************/

OGRErr OGRMiraMonLayer::TranslateFieldsValuesToMM(OGRFeature *poFeature)

{
    if (poFeatureDefn->GetFieldCount() == 0)
    {
        // MiraMon have private DataBase records
        hMMFeature.nNumMRecords = 1;
        return OGRERR_NONE;
    }

    MM_EXT_DBF_N_MULTIPLE_RECORDS nIRecord;
    int nNumFields = poFeatureDefn->GetFieldCount();
    MM_EXT_DBF_N_MULTIPLE_RECORDS nNumRecords;
    hMMFeature.nNumMRecords = 0;

    for (int iField = 0; iField < nNumFields; iField++)
    {
        OGRFieldType eFType = poFeatureDefn->GetFieldDefn(iField)->GetType();
        const char *pszRawValue = poFeature->GetFieldAsString(iField);

        if (eFType == OFTStringList)
        {
            char **panValues = poFeature->GetFieldAsStringList(iField);
            nNumRecords = CSLCount(panValues);
            if (nNumRecords == 0)
                nNumRecords++;
            hMMFeature.nNumMRecords =
                max_function(hMMFeature.nNumMRecords, nNumRecords);
            if (MMResizeMiraMonRecord(
                    &hMMFeature.pRecords, &hMMFeature.nMaxMRecords,
                    hMMFeature.nNumMRecords, MM_INC_NUMBER_OF_RECORDS,
                    hMMFeature.nNumMRecords))
                return OGRERR_NOT_ENOUGH_MEMORY;

            for (nIRecord = 0; nIRecord < hMMFeature.nNumMRecords; nIRecord++)
            {
                hMMFeature.pRecords[nIRecord].nNumField =
                    poFeatureDefn->GetFieldCount();

                if (MMResizeMiraMonFieldValue(
                        &(hMMFeature.pRecords[nIRecord].pField),
                        &hMMFeature.pRecords[nIRecord].nMaxField,
                        hMMFeature.pRecords[nIRecord].nNumField,
                        MM_INC_NUMBER_OF_FIELDS,
                        hMMFeature.pRecords[nIRecord].nNumField))
                    return OGRERR_NOT_ENOUGH_MEMORY;

                if (phMiraMonLayer->nCharSet != MM_JOC_CARAC_UTF8_DBF)
                {
                    // MiraMon encoding is ISO 8859-1 (Latin1) -> Recode from UTF-8
                    char *pszString = CPLRecode(
                        panValues[nIRecord], CPL_ENC_UTF8, CPL_ENC_ISO8859_1);
                    if (MM_SecureCopyStringFieldValue(
                            &hMMFeature.pRecords[nIRecord]
                                 .pField[iField]
                                 .pDinValue,
                            pszString,
                            &hMMFeature.pRecords[nIRecord]
                                 .pField[iField]
                                 .nNumDinValue))
                    {
                        CPLFree(pszString);
                        return OGRERR_NOT_ENOUGH_MEMORY;
                    }
                    CPLFree(pszString);
                }
                else
                {
                    if (MM_SecureCopyStringFieldValue(
                            &hMMFeature.pRecords[nIRecord]
                                 .pField[iField]
                                 .pDinValue,
                            panValues[nIRecord],
                            &hMMFeature.pRecords[nIRecord]
                                 .pField[iField]
                                 .nNumDinValue))
                        return OGRERR_NOT_ENOUGH_MEMORY;
                }
                hMMFeature.pRecords[nIRecord].pField[iField].bIsValid = 1;
            }
        }
        else if (eFType == OFTIntegerList)
        {
            int nCount = 0;
            const int *panValues =
                poFeature->GetFieldAsIntegerList(iField, &nCount);

            nNumRecords = nCount;
            if (nNumRecords == 0)
                nNumRecords++;
            hMMFeature.nNumMRecords =
                max_function(hMMFeature.nNumMRecords, nNumRecords);
            if (MMResizeMiraMonRecord(
                    &hMMFeature.pRecords, &hMMFeature.nMaxMRecords,
                    hMMFeature.nNumMRecords, MM_INC_NUMBER_OF_RECORDS,
                    hMMFeature.nNumMRecords))
                return OGRERR_NOT_ENOUGH_MEMORY;

            for (nIRecord = 0; nIRecord < hMMFeature.nNumMRecords; nIRecord++)
            {
                hMMFeature.pRecords[nIRecord].nNumField = nNumFields;

                if (MMResizeMiraMonFieldValue(
                        &(hMMFeature.pRecords[nIRecord].pField),
                        &hMMFeature.pRecords[nIRecord].nMaxField,
                        hMMFeature.pRecords[nIRecord].nNumField,
                        MM_INC_NUMBER_OF_FIELDS,
                        hMMFeature.pRecords[nIRecord].nNumField))
                    return OGRERR_NOT_ENOUGH_MEMORY;

                hMMFeature.pRecords[nIRecord].pField[iField].dValue =
                    panValues[nIRecord];

                if (MM_SecureCopyStringFieldValue(
                        &hMMFeature.pRecords[nIRecord].pField[iField].pDinValue,
                        MMGetNFieldValue(pszRawValue, nIRecord),
                        &hMMFeature.pRecords[nIRecord]
                             .pField[iField]
                             .nNumDinValue))
                    return OGRERR_NOT_ENOUGH_MEMORY;

                hMMFeature.pRecords[nIRecord].pField[iField].bIsValid = 1;
            }
        }
        else if (eFType == OFTInteger64List)
        {
            int nCount = 0;
            const GIntBig *panValues =
                poFeature->GetFieldAsInteger64List(iField, &nCount);
            nNumRecords = nCount;
            if (nNumRecords == 0)
                nNumRecords++;
            hMMFeature.nNumMRecords =
                max_function(hMMFeature.nNumMRecords, nNumRecords);
            if (MMResizeMiraMonRecord(
                    &hMMFeature.pRecords, &hMMFeature.nMaxMRecords,
                    hMMFeature.nNumMRecords, MM_INC_NUMBER_OF_RECORDS,
                    hMMFeature.nNumMRecords))
                return OGRERR_NOT_ENOUGH_MEMORY;

            for (nIRecord = 0; nIRecord < hMMFeature.nNumMRecords; nIRecord++)
            {
                hMMFeature.pRecords[nIRecord].nNumField = nNumFields;

                if (MMResizeMiraMonFieldValue(
                        &(hMMFeature.pRecords[nIRecord].pField),
                        &hMMFeature.pRecords[nIRecord].nMaxField,
                        hMMFeature.pRecords[nIRecord].nNumField,
                        MM_INC_NUMBER_OF_FIELDS,
                        hMMFeature.pRecords[nIRecord].nNumField))
                    return OGRERR_NOT_ENOUGH_MEMORY;

                hMMFeature.pRecords[nIRecord].pField[iField].iValue =
                    panValues[nIRecord];
                if (MM_SecureCopyStringFieldValue(
                        &hMMFeature.pRecords[nIRecord].pField[iField].pDinValue,
                        MMGetNFieldValue(pszRawValue, nIRecord),
                        &hMMFeature.pRecords[nIRecord]
                             .pField[iField]
                             .nNumDinValue))
                    return OGRERR_NOT_ENOUGH_MEMORY;
                hMMFeature.pRecords[nIRecord].pField[iField].bIsValid = 1;
            }
        }
        else if (eFType == OFTRealList)
        {
            int nCount = 0;
            const double *panValues =
                poFeature->GetFieldAsDoubleList(iField, &nCount);
            nNumRecords = nCount;
            if (nNumRecords == 0)
                nNumRecords++;
            hMMFeature.nNumMRecords =
                max_function(hMMFeature.nNumMRecords, nNumRecords);
            if (MMResizeMiraMonRecord(
                    &hMMFeature.pRecords, &hMMFeature.nMaxMRecords,
                    hMMFeature.nNumMRecords, MM_INC_NUMBER_OF_RECORDS,
                    hMMFeature.nNumMRecords))
                return OGRERR_NOT_ENOUGH_MEMORY;

            for (nIRecord = 0; nIRecord < hMMFeature.nNumMRecords; nIRecord++)
            {
                hMMFeature.pRecords[nIRecord].nNumField = iField;

                if (MMResizeMiraMonFieldValue(
                        &(hMMFeature.pRecords[nIRecord].pField),
                        &hMMFeature.pRecords[nIRecord].nMaxField,
                        hMMFeature.pRecords[nIRecord].nNumField,
                        MM_INC_NUMBER_OF_FIELDS,
                        hMMFeature.pRecords[nIRecord].nNumField))
                    return OGRERR_NOT_ENOUGH_MEMORY;

                hMMFeature.pRecords[nIRecord].pField[iField].dValue =
                    panValues[nIRecord];
                if (MM_SecureCopyStringFieldValue(
                        &hMMFeature.pRecords[nIRecord].pField[iField].pDinValue,
                        MMGetNFieldValue(pszRawValue, nIRecord),
                        &hMMFeature.pRecords[nIRecord]
                             .pField[iField]
                             .nNumDinValue))
                    return OGRERR_NOT_ENOUGH_MEMORY;
                hMMFeature.pRecords[nIRecord].pField[iField].bIsValid = 1;
            }
        }
        else if (eFType == OFTString)
        {
            hMMFeature.nNumMRecords = max_function(hMMFeature.nNumMRecords, 1);
            hMMFeature.pRecords[0].nNumField = nNumFields;
            if (MMResizeMiraMonFieldValue(&(hMMFeature.pRecords[0].pField),
                                          &hMMFeature.pRecords[0].nMaxField,
                                          hMMFeature.pRecords[0].nNumField,
                                          MM_INC_NUMBER_OF_FIELDS,
                                          hMMFeature.pRecords[0].nNumField))
                return OGRERR_NOT_ENOUGH_MEMORY;

            if (phMiraMonLayer->nCharSet != MM_JOC_CARAC_UTF8_DBF)
            {
                // MiraMon encoding is ISO 8859-1 (Latin1) -> Recode from UTF-8
                char *pszString =
                    CPLRecode(pszRawValue, CPL_ENC_UTF8, CPL_ENC_ISO8859_1);
                if (MM_SecureCopyStringFieldValue(
                        &hMMFeature.pRecords[0].pField[iField].pDinValue,
                        pszString,
                        &hMMFeature.pRecords[0].pField[iField].nNumDinValue))
                {
                    CPLFree(pszString);
                    return OGRERR_NOT_ENOUGH_MEMORY;
                }
                CPLFree(pszString);
            }
            else
            {
                if (MM_SecureCopyStringFieldValue(
                        &hMMFeature.pRecords[0].pField[iField].pDinValue,
                        pszRawValue,
                        &hMMFeature.pRecords[0].pField[iField].nNumDinValue))
                {
                    return OGRERR_NOT_ENOUGH_MEMORY;
                }
            }
            hMMFeature.pRecords[0].pField[iField].bIsValid = 1;
        }
        else if (eFType == OFTDate)
        {
            char szDate[15];

            hMMFeature.nNumMRecords = max_function(hMMFeature.nNumMRecords, 1);
            hMMFeature.pRecords[0].nNumField = nNumFields;
            if (MMResizeMiraMonFieldValue(&(hMMFeature.pRecords[0].pField),
                                          &hMMFeature.pRecords[0].nMaxField,
                                          hMMFeature.pRecords[0].nNumField,
                                          MM_INC_NUMBER_OF_FIELDS,
                                          hMMFeature.pRecords[0].nNumField))
                return OGRERR_NOT_ENOUGH_MEMORY;

            const OGRField *poField = poFeature->GetRawFieldRef(iField);
            if (poField->Date.Year >= 0)
                snprintf(szDate, sizeof(szDate), "%04d%02d%02d",
                         poField->Date.Year, poField->Date.Month,
                         poField->Date.Day);
            else
                snprintf(szDate, sizeof(szDate), "%04d%02d%02d", 0, 0, 0);

            if (MM_SecureCopyStringFieldValue(
                    &hMMFeature.pRecords[0].pField[iField].pDinValue, szDate,
                    &hMMFeature.pRecords[0].pField[iField].nNumDinValue))
                return OGRERR_NOT_ENOUGH_MEMORY;
            hMMFeature.pRecords[0].pField[iField].bIsValid = 1;
        }
        else if (eFType == OFTTime || eFType == OFTDateTime)
        {
            hMMFeature.nNumMRecords = max_function(hMMFeature.nNumMRecords, 1);
            hMMFeature.pRecords[0].nNumField = nNumFields;
            if (MMResizeMiraMonFieldValue(&(hMMFeature.pRecords[0].pField),
                                          &hMMFeature.pRecords[0].nMaxField,
                                          hMMFeature.pRecords[0].nNumField,
                                          MM_INC_NUMBER_OF_FIELDS,
                                          hMMFeature.pRecords[0].nNumField))
                return OGRERR_NOT_ENOUGH_MEMORY;

            // MiraMon encoding is ISO 8859-1 (Latin1) -> Recode from UTF-8
            if (MM_SecureCopyStringFieldValue(
                    &hMMFeature.pRecords[0].pField[iField].pDinValue,
                    pszRawValue,
                    &hMMFeature.pRecords[0].pField[iField].nNumDinValue))
                return OGRERR_NOT_ENOUGH_MEMORY;

            hMMFeature.pRecords[0].pField[iField].bIsValid = 1;
        }
        else if (eFType == OFTInteger)
        {
            hMMFeature.nNumMRecords = max_function(hMMFeature.nNumMRecords, 1);
            hMMFeature.pRecords[0].nNumField = nNumFields;
            if (MMResizeMiraMonFieldValue(&(hMMFeature.pRecords[0].pField),
                                          &hMMFeature.pRecords[0].nMaxField,
                                          hMMFeature.pRecords[0].nNumField,
                                          MM_INC_NUMBER_OF_FIELDS,
                                          hMMFeature.pRecords[0].nNumField))
                return OGRERR_NOT_ENOUGH_MEMORY;

            hMMFeature.pRecords[0].pField[iField].dValue =
                poFeature->GetFieldAsInteger(iField);
            if (MM_SecureCopyStringFieldValue(
                    &hMMFeature.pRecords[0].pField[iField].pDinValue,
                    pszRawValue,
                    &hMMFeature.pRecords[0].pField[iField].nNumDinValue))
                return OGRERR_NOT_ENOUGH_MEMORY;
            hMMFeature.pRecords[0].pField[iField].bIsValid = 1;
        }
        else if (eFType == OFTInteger64)
        {
            hMMFeature.nNumMRecords = max_function(hMMFeature.nNumMRecords, 1);
            hMMFeature.pRecords[0].nNumField = nNumFields;
            if (MMResizeMiraMonFieldValue(&(hMMFeature.pRecords[0].pField),
                                          &hMMFeature.pRecords[0].nMaxField,
                                          hMMFeature.pRecords[0].nNumField,
                                          MM_INC_NUMBER_OF_FIELDS,
                                          hMMFeature.pRecords[0].nNumField))
                return OGRERR_NOT_ENOUGH_MEMORY;

            hMMFeature.pRecords[0].pField[iField].iValue =
                poFeature->GetFieldAsInteger64(iField);
            if (MM_SecureCopyStringFieldValue(
                    &hMMFeature.pRecords[0].pField[iField].pDinValue,
                    poFeature->GetFieldAsString(iField),
                    &hMMFeature.pRecords[0].pField[iField].nNumDinValue))
                return OGRERR_NOT_ENOUGH_MEMORY;
            hMMFeature.pRecords[0].pField[iField].bIsValid = 1;
        }
        else if (eFType == OFTReal)
        {
            hMMFeature.nNumMRecords = max_function(hMMFeature.nNumMRecords, 1);
            hMMFeature.pRecords[0].nNumField = nNumFields;
            if (MMResizeMiraMonFieldValue(&(hMMFeature.pRecords[0].pField),
                                          &hMMFeature.pRecords[0].nMaxField,
                                          hMMFeature.pRecords[0].nNumField,
                                          MM_INC_NUMBER_OF_FIELDS,
                                          hMMFeature.pRecords[0].nNumField))
                return OGRERR_NOT_ENOUGH_MEMORY;

            hMMFeature.pRecords[0].pField[iField].dValue =
                poFeature->GetFieldAsDouble(iField);
            if (MM_SecureCopyStringFieldValue(
                    &hMMFeature.pRecords[0].pField[iField].pDinValue,
                    poFeature->GetFieldAsString(iField),
                    &hMMFeature.pRecords[0].pField[iField].nNumDinValue))
                return OGRERR_NOT_ENOUGH_MEMORY;
            hMMFeature.pRecords[0].pField[iField].bIsValid = 1;
        }
        else
        {
            CPLError(CE_Failure, CPLE_NotSupported,
                     "MiraMon: Field type %d not processed by MiraMon\n",
                     eFType);
            return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
        }
    }

    return OGRERR_NONE;
}

/****************************************************************************/
/*                             GetLayerDefn()                               */
/*                                                                          */
/****************************************************************************/
OGRFeatureDefn *OGRMiraMonLayer::GetLayerDefn()
{
    return poFeatureDefn;
}

/****************************************************************************/
/*                             GetExtent()                                  */
/*                                                                          */
/*      Fetch extent of the data currently stored in the dataset.           */
/*      The bForce flag has no effect on SHO files since that value         */
/*      is always in the header.                                            */
/****************************************************************************/

OGRErr OGRMiraMonLayer::GetExtent(OGREnvelope *psExtent, int bForce)

{
    if (phMiraMonLayer)
    {
        psExtent->MinX = phMiraMonLayer->TopHeader.hBB.dfMinX;
        psExtent->MaxX = phMiraMonLayer->TopHeader.hBB.dfMaxX;
        psExtent->MinY = phMiraMonLayer->TopHeader.hBB.dfMinY;
        psExtent->MaxY = phMiraMonLayer->TopHeader.hBB.dfMaxY;
    }
    else
    {
        if (!bForce)
            return OGRERR_FAILURE;
    }

    return OGRERR_NONE;
}

/****************************************************************************/
/*                           TestCapability()                               */
/****************************************************************************/

int OGRMiraMonLayer::TestCapability(const char *pszCap)

{
    if (EQUAL(pszCap, OLCRandomRead))
        return TRUE;

    if (EQUAL(pszCap, OLCSequentialWrite))
        return TRUE;

    if (EQUAL(pszCap, OLCFastFeatureCount))
        return TRUE;

    if (EQUAL(pszCap, OLCFastGetExtent))
        return TRUE;

    if (EQUAL(pszCap, OLCCreateField))
        return TRUE;

    if (EQUAL(pszCap, OLCFastFeatureCount))
        return TRUE;

    if (EQUAL(pszCap, OLCZGeometries))
        return TRUE;

    if (EQUAL(pszCap, OLCStringsAsUTF8))
        return TRUE;

    return FALSE;
}

/****************************************************************************/
/*                            CreateField()                                 */
/****************************************************************************/

OGRErr OGRMiraMonLayer::CreateField(const OGRFieldDefn *poField, int bApproxOK)

{
    if (!bUpdate)
    {
        CPLError(CE_Failure, CPLE_NoWriteAccess,
                 "\nCannot create fields on a read-only dataset.");
        return OGRERR_FAILURE;
    }

    switch (poField->GetType())
    {
        case OFTInteger:
        case OFTIntegerList:
        case OFTInteger64:
        case OFTInteger64List:
        case OFTReal:
        case OFTRealList:
        case OFTString:
        case OFTStringList:
        case OFTDate:
            poFeatureDefn->AddFieldDefn(poField);
            return OGRERR_NONE;
        default:
            if (!bApproxOK)
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "\nField %s is of an unsupported type: %s.",
                         poField->GetNameRef(),
                         poField->GetFieldTypeName(poField->GetType()));
                return OGRERR_FAILURE;
            }
            else
            {
                OGRFieldDefn oModDef(poField);
                oModDef.SetType(OFTString);
                poFeatureDefn->AddFieldDefn(poField);
                return OGRERR_NONE;
            }
    }
}

/************************************************************************/
/*                            AddToFileList()                           */
/************************************************************************/

void OGRMiraMonLayer::AddToFileList(CPLStringList &oFileList)
{
    if (!phMiraMonLayer)
        return;

    char szAuxFile[MM_CPL_PATH_BUF_SIZE];

    oFileList.AddString(
        VSIGetCanonicalFilename(phMiraMonLayer->pszSrcLayerName));
    char *pszMMExt =
        CPLStrdup(CPLGetExtension(phMiraMonLayer->pszSrcLayerName));

    if (phMiraMonLayer->bIsPoint)
    {
        // As it's explicit on documentation a point has also two more files:

        // FILE_NAME_WITHOUT_EXTENSION.pnt --> FILE_NAME_WITHOUT_EXTENSION + T.rel
        CPLStrlcpy(szAuxFile, CPLGetBasename(phMiraMonLayer->pszSrcLayerName),
                   MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'p') ? "T.rel" : "T.REL",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.pnt --> FILE_NAME_WITHOUT_EXTENSION + T.dbf
        CPLStrlcpy(szAuxFile, CPLGetBasename(phMiraMonLayer->pszSrcLayerName),
                   MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'p') ? "T.dbf" : "T.DBF",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr)));
    }
    else if (phMiraMonLayer->bIsArc && !phMiraMonLayer->bIsPolygon)
    {
        // As it's explicit on documentation a point has also five more files:

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + A.rel
        CPLStrlcpy(szAuxFile, CPLGetBasename(phMiraMonLayer->pszSrcLayerName),
                   MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'a') ? "A.rel" : "A.REL",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + A.dbf
        CPLStrlcpy(szAuxFile, CPLGetBasename(phMiraMonLayer->pszSrcLayerName),
                   MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'a') ? "A.dbf" : "A.DBF",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + .nod
        CPLStrlcpy(szAuxFile, CPLGetBasename(phMiraMonLayer->pszSrcLayerName),
                   MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'a') ? ".nod" : ".NOD",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + N.rel
        CPLStrlcpy(szAuxFile, CPLGetBasename(phMiraMonLayer->pszSrcLayerName),
                   MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'a') ? "N.rel" : "N.REL",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + N.dbf
        CPLStrlcpy(szAuxFile, CPLGetBasename(phMiraMonLayer->pszSrcLayerName),
                   MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'a') ? "N.dbf" : "N.DBF",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr)));
    }
    else if (phMiraMonLayer->bIsPolygon)
    {
        // As it's explicit on documentation a point has also eight more files:
        const char *szCompleteArcFileName;
        char szArcFileName[MM_CPL_PATH_BUF_SIZE];

        // FILE_NAME_WITHOUT_EXTENSION.pol --> FILE_NAME_WITHOUT_EXTENSION + P.rel
        CPLStrlcpy(szAuxFile, CPLGetBasename(phMiraMonLayer->pszSrcLayerName),
                   MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'p') ? "P.rel" : "P.REL",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr)));

        // The name of the arc is in THIS metadata file
        char *pszArcLayerName = MMReturnValueFromSectionINIFile(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr),
            SECTION_OVVW_ASPECTES_TECNICS, KEY_ArcSource);
        if (!pszArcLayerName)
        {
            CPLFree(pszMMExt);
            return;  //Some files are missing
        }
        CPLStrlcpy(szArcFileName, pszArcLayerName, MM_CPL_PATH_BUF_SIZE);

        MM_RemoveInitial_and_FinalQuotationMarks(szArcFileName);

        // If extension is not specified ".arc" will be used
        if (!pszArcLayerName ||
            MMIsEmptyString(CPLGetExtension(pszArcLayerName)))
            CPLStrlcat(szArcFileName, (pszMMExt[0] == 'p') ? ".arc" : ".ARC",
                       MM_CPL_PATH_BUF_SIZE);

        CPLFree(pszArcLayerName);

        szCompleteArcFileName =
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szArcFileName, nullptr);

        // The arc that has the coordinates of the polygon
        oFileList.AddStringDirectly(
            VSIGetCanonicalFilename(szCompleteArcFileName));

        // FILE_NAME_WITHOUT_EXTENSION.pol --> FILE_NAME_WITHOUT_EXTENSION + P.dbf
        CPLStrlcpy(szAuxFile, CPLGetBasename(phMiraMonLayer->pszSrcLayerName),
                   MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'p') ? "P.dbf" : "P.DBF",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(
            CPLFormFilename(CPLGetDirname(phMiraMonLayer->pszSrcLayerName),
                            szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + A.rel
        const char *pszBaseArcName = CPLGetBasename(szCompleteArcFileName);
        CPLStrlcpy(szAuxFile, pszBaseArcName, MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'p') ? "A.rel" : "A.REL",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(CPLFormFilename(
            CPLGetDirname(szCompleteArcFileName), szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + A.dbf
        CPLStrlcpy(szAuxFile, pszBaseArcName, MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'p') ? "A.dbf" : "A.DBF",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(CPLFormFilename(
            CPLGetDirname(szCompleteArcFileName), szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + .nod
        CPLStrlcpy(szAuxFile, pszBaseArcName, MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'p') ? ".nod" : ".NOD",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(CPLFormFilename(
            CPLGetDirname(szCompleteArcFileName), szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + N.rel
        CPLStrlcpy(szAuxFile, pszBaseArcName, MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'p') ? "N.rel" : "N.REL",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(CPLFormFilename(
            CPLGetDirname(szCompleteArcFileName), szAuxFile, nullptr)));

        // FILE_NAME_WITHOUT_EXTENSION.arc --> FILE_NAME_WITHOUT_EXTENSION + N.dbf
        CPLStrlcpy(szAuxFile, pszBaseArcName, MM_CPL_PATH_BUF_SIZE);
        CPLStrlcat(szAuxFile, (pszMMExt[0] == 'p') ? "N.dbf" : "N.DBF",
                   MM_CPL_PATH_BUF_SIZE);
        oFileList.AddStringDirectly(VSIGetCanonicalFilename(CPLFormFilename(
            CPLGetDirname(szCompleteArcFileName), szAuxFile, nullptr)));
    }
    CPLFree(pszMMExt);
}

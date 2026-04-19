/******************************************************************************
 *
 * Project:  S-101 driver
 * Purpose:  Implements OGRS101Reader
 * Author:   Even Rouault <even dot rouault at spatialys.com>
 *
 ******************************************************************************
 * Copyright (c) 2026, Even Rouault <even dot rouault at spatialys.com>
 *
 * SPDX-License-Identifier: MIT
 ****************************************************************************/

#include "ogr_s101.h"
#include "ogrs101readerconstants.h"

#include <memory>

/************************************************************************/
/*                       GetMultiPointLayerName()                       */
/************************************************************************/

/* static */ std::string
OGRS101Reader::GetMultiPointLayerName(const OGRSpatialReference &oSRS)
{
    return oSRS.GetAxesCount() == 2
               ? "MultiPoint2D"
               : CPLSPrintf("MultiPoint3D_%s", LaunderCRSName(oSRS).c_str());
}

/************************************************************************/
/*                    CreateMultiPointFeatureDefns()                    */
/************************************************************************/

/** Create the feature definition(s) for the MultiPoint layer(s)
 *
 * There is a layer per CRS used by multipoints.
 */
bool OGRS101Reader::CreateMultiPointFeatureDefns()
{
    bool bError = false;

    m_oMapCRSIdToMultiPointRecordIdx =
        CreateMapCRSIdToRecordIdxForMultiPoints(bError);
    if (bError)
        return false;
    for (const auto &[nCRSId, anRecordIdx] : m_oMapCRSIdToMultiPointRecordIdx)
    {
        const auto &oSRS = m_oMapSRS[nCRSId];
        const bool bIs2D = nCRSId == HORIZONTAL_CRS_ID;
        auto poFDefn = OGRFeatureDefnRefCountedPtr::makeInstance(
            GetMultiPointLayerName(oSRS).c_str());
        poFDefn->SetGeomType(bIs2D ? wkbMultiPoint : wkbMultiPoint25D);
        poFDefn->GetGeomFieldDefn(0)->SetSpatialRef(
            OGRSpatialReferenceRefCountedPtr::makeClone(&oSRS).get());
        {
            OGRFieldDefn oFieldDefn(OGR_FIELD_NAME_RECORD_ID, OFTInteger);
            poFDefn->AddFieldDefn(&oFieldDefn);
        }
        {
            OGRFieldDefn oFieldDefn(OGR_FIELD_NAME_RECORD_VERSION, OFTInteger);
            poFDefn->AddFieldDefn(&oFieldDefn);
        }
        if (!InferFeatureDefn(m_oMultiPointRecordIndex, MRID_FIELD, INAS_FIELD,
                              anRecordIdx, *poFDefn, m_oMapFieldDomains))
        {
            return false;
        }
        m_oMapMultiPointFeatureDefn[nCRSId] = std::move(poFDefn);
    }

    return true;
}

/************************************************************************/
/*                    GetCRSIdForMultiPointRecord()                     */
/************************************************************************/

/** Return the CRS id for a given MultiPoint record, or INVALID_CRS_ID on error */
OGRS101Reader::CRSId
OGRS101Reader::GetCRSIdForMultiPointRecord(const DDFRecord *poRecord,
                                           int iRecord, int nRecordID) const
{
    if (nRecordID < 0)
        nRecordID = poRecord->GetIntSubfield(MRID_FIELD, 0, RCID_SUBFIELD, 0);

    const auto GetErrorContext = [iRecord, nRecordID]()
    {
        if (iRecord >= 0)
            return CPLSPrintf("Record index=%d of MRID", iRecord);
        else
            return CPLSPrintf("Record ID=%d of MRID", nRecordID);
    };

    if (poRecord->FindField(C3IL_FIELD))
    {
        const CRSId nVCID =
            poRecord->GetIntSubfield(C3IL_FIELD, 0, VCID_SUBFIELD, 0);
        if (nVCID == HORIZONTAL_CRS_ID)
        {
            CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(
                CPLSPrintf("%s: VCID subfield = %d of C3IL "
                           "field points to a non-3D CRS.",
                           GetErrorContext(), static_cast<int>(nVCID))));
            return INVALID_CRS_ID;
        }
        else if (!cpl::contains(m_oMapSRS, nVCID))
        {
            CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(
                CPLSPrintf("%s: Unknown value %d for VCID subfield of C3IL "
                           "field.",
                           GetErrorContext(), static_cast<int>(nVCID))));
            return INVALID_CRS_ID;
        }
        else
        {
            return nVCID;
        }
    }
    else if (poRecord->FindField(C2IL_FIELD))
    {
        return HORIZONTAL_CRS_ID;
    }
    else
    {
        CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(
            CPLSPrintf("%s: No C2IL or C3IL field found.", GetErrorContext())));
        return INVALID_CRS_ID;
    }
}

/************************************************************************/
/*              CreateMapCRSIdToRecordIdxForMultiPoints()               */
/************************************************************************/

/** Browse through m_oMultiPointRecordIndex to identify which record belongs to
 * each CRS and create a map from each CRS id to the record indices that use
 * it.
 */
std::map<OGRS101Reader::CRSId, std::vector<int>>
OGRS101Reader::CreateMapCRSIdToRecordIdxForMultiPoints(bool &bError) const
{
    std::map<OGRS101Reader::CRSId, std::vector<int>> map;

    const int nRecords = m_oMultiPointRecordIndex.GetCount();
    for (int iRecord = 0; iRecord < nRecords; ++iRecord)
    {
        const auto poRecord = m_oMultiPointRecordIndex.GetByIndex(iRecord);
        const CRSId nCRSId = GetCRSIdForMultiPointRecord(poRecord, iRecord,
                                                         /* nRecordID = */ -1);
        if (nCRSId == INVALID_CRS_ID)
        {
            if (m_bStrict)
            {
                bError = true;
                return {};
            }
        }
        else
        {
            map[nCRSId].push_back(iRecord);
        }
    }

    return map;
}

/************************************************************************/
/*                       ReadMultiPointGeometry()                       */
/************************************************************************/

std::unique_ptr<OGRMultiPoint>
OGRS101Reader::ReadMultiPointGeometry(const DDFRecord *poRecord, int iRecord,
                                      int nRecordID,
                                      const OGRSpatialReference *poSRS) const
{
    const bool bIs3D = poRecord->FindField(C3IL_FIELD) != nullptr;
    const char *pszCoordFieldName = bIs3D ? C3IL_FIELD : C2IL_FIELD;
    const auto apoCoordFields = poRecord->GetFields(pszCoordFieldName);
    if (apoCoordFields.empty())
        return nullptr;

    auto poMP = std::make_unique<OGRMultiPoint>();
    poMP->assignSpatialReference(poSRS);
    for (const auto *poCoordField : apoCoordFields)
    {
        const int nCoordCount =
            bIs3D && poCoordField->GetParts().size() == 2
                ? poCoordField->GetParts()[1]->GetRepeatCount()
                : poCoordField->GetRepeatCount();

        for (int iPnt = 0; iPnt < nCoordCount; ++iPnt)
        {
            auto poPoint = ReadPointGeometryInternal(
                poRecord, iRecord, nRecordID, iPnt, poSRS, bIs3D, poCoordField,
                MRID_FIELD);
            if (!poPoint)
                return nullptr;
            poMP->addGeometry(std::move(poPoint));
        }
    }
    return poMP;
}

/************************************************************************/
/*                       FillFeatureMultiPoint()                        */
/************************************************************************/

/** Fill the content of the provided feature from the identified record
 * (of m_oMultiPointRecordIndex).
 */
bool OGRS101Reader::FillFeatureMultiPoint(const DDFRecordIndex &oIndex,
                                          int iRecord,
                                          OGRFeature &oFeature) const
{
    const auto poRecord = oIndex.GetByIndex(iRecord);
    CPLAssert(poRecord);

    const OGRSpatialReference *poSRS =
        oFeature.GetDefnRef()->GetGeomFieldDefn(0)->GetSpatialRef();
    auto poMP =
        ReadMultiPointGeometry(poRecord, iRecord, /* nRecordID = */ -1, poSRS);
    if (poMP)
    {
        oFeature.SetGeometry(std::move(poMP));
    }
    else if (m_bStrict)
        return false;

    return FillFeatureAttributes(oIndex, iRecord, INAS_FIELD, oFeature) &&
           FillFeatureWithNonAttrAssocSubfields(poRecord, iRecord, INAS_FIELD,
                                                oFeature);
}

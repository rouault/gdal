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
#include <set>

/************************************************************************/
/*                  CreateCompositeCurveFeatureDefn()                   */
/************************************************************************/

/** Create the feature definition for the CompositeCurve layer
 */
bool OGRS101Reader::CreateCompositeCurveFeatureDefn()
{
    if (m_oCompositeCurveRecordIndex.GetCount() > 0)
    {
        m_poFeatureDefnCompositeCurve =
            OGRFeatureDefnRefCountedPtr::makeInstance(
                OGR_LAYER_NAME_COMPOSITE_CURVE);
        m_poFeatureDefnCompositeCurve->SetGeomType(wkbLineString);
        auto oSRSIter = m_oMapSRS.find(HORIZONTAL_CRS_ID);
        if (oSRSIter != m_oMapSRS.end())
        {
            m_poFeatureDefnCompositeCurve->GetGeomFieldDefn(0)->SetSpatialRef(
                OGRSpatialReferenceRefCountedPtr::makeClone(&oSRSIter->second)
                    .get());
        }
        {
            OGRFieldDefn oFieldDefn(OGR_FIELD_NAME_RECORD_ID, OFTInteger);
            m_poFeatureDefnCompositeCurve->AddFieldDefn(&oFieldDefn);
        }
        {
            OGRFieldDefn oFieldDefn(OGR_FIELD_NAME_RECORD_VERSION, OFTInteger);
            m_poFeatureDefnCompositeCurve->AddFieldDefn(&oFieldDefn);
        }
        if (!InferFeatureDefn(m_oCompositeCurveRecordIndex, CCID_FIELD,
                              INAS_FIELD, {}, *m_poFeatureDefnCompositeCurve,
                              m_oMapFieldDomains))
        {
            return false;
        }
    }

    return true;
}

/************************************************************************/
/*                     ReadCompositeCurveGeometry()                     */
/************************************************************************/

std::unique_ptr<OGRLineString>
OGRS101Reader::ReadCompositeCurveGeometryInternal(
    const DDFRecord *poRecord, int iRecord, int nRecordID,
    const OGRSpatialReference *poSRS,
    std::set<int> &oSetAlreadyVisitedCompositeCurveRecords) const
{
    if (nRecordID < 0)
        nRecordID = poRecord->GetIntSubfield(CCID_FIELD, 0, RCID_SUBFIELD, 0);

    const auto GetErrorContext = [iRecord, nRecordID]()
    {
        if (iRecord >= 0)
            return CPLSPrintf("Record index=%d of CCID", iRecord);
        else
            return CPLSPrintf("Record ID=%d of CCID", nRecordID);
    };

    // The spec mentions that a composite curve may only refer to
    // another composite curve whose definition occurs before in the
    // file, which, if respected, effectively prevents circular
    // referencing from happening.
    // It is not practical for us to take note of the appearance order,
    // so use a set of record *ids* to detect that (so we are a bit
    // more permissive)
    if (!oSetAlreadyVisitedCompositeCurveRecords.insert(nRecordID).second)
    {
        CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(CPLSPrintf(
            "%s: circular dependency on (RCNM=%d, RCID=%d) while "
            "resolving composite curve geometry.",
            GetErrorContext(), static_cast<int>(RECORD_NAME_COMPOSITE_CURVE),
            nRecordID)));
        return nullptr;
    }

    const auto apoCUCOFields = poRecord->GetFields(CUCO_FIELD);
    if (apoCUCOFields.empty())
    {
        CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(
            CPLSPrintf("%s: no CUCO field", GetErrorContext())));
        return nullptr;
    }

    auto poLS = std::make_unique<OGRLineString>();
    for (const auto *poCUCOField : apoCUCOFields)
    {
        const int nParts = poCUCOField->GetRepeatCount();
        for (int iPart = 0; iPart < nParts; ++iPart)
        {
            const auto GetIntSubfield =
                [poRecord, poCUCOField, iPart](const char *pszSubFieldName)
            {
                return poRecord->GetIntSubfield(poCUCOField, pszSubFieldName,
                                                iPart);
            };

            const RecordName nRRNM = GetIntSubfield(RRNM_SUBFIELD);
            if (nRRNM != RECORD_NAME_CURVE &&
                nRRNM != RECORD_NAME_COMPOSITE_CURVE)
            {
                CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(CPLSPrintf(
                    "%s: Invalid value for RRNM "
                    "subfield of %d instance of CUCO field: "
                    "got %d, expected %d or %d.",
                    GetErrorContext(), iPart, static_cast<int>(nRRNM),
                    static_cast<int>(RECORD_NAME_CURVE),
                    static_cast<int>(RECORD_NAME_COMPOSITE_CURVE))));
                return nullptr;
            }

            bool bReverse = false;
            const int nORNT = GetIntSubfield(ORNT_SUBFIELD);
            if (nORNT == ORNT_REVERSE)
            {
                bReverse = true;
            }
            else if (nORNT != ORNT_FORWARD)
            {
                CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(
                    CPLSPrintf("%s: Invalid value for ORNT "
                               "subfield of %d instance of CUCO field: "
                               "got %d, expected %d or %d.",
                               GetErrorContext(), iPart, nORNT, ORNT_FORWARD,
                               ORNT_REVERSE)));
                return nullptr;
            }

            const int nRRID = GetIntSubfield(RRID_SUBFIELD);
            std::unique_ptr<OGRLineString> poCurvePart;
            if (nRRNM == RECORD_NAME_CURVE)
            {
                const auto poCurveRecord =
                    m_oCurveRecordIndex.FindRecord(nRRID);
                if (!poCurveRecord)
                {
                    CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(CPLSPrintf(
                        "%s: Value (RRNM=%d, RRID=%d) "
                        "of instance %d of CUCO field does not point to an "
                        "existing record.",
                        GetErrorContext(), static_cast<int>(nRRNM), nRRID,
                        iPart)));
                    return nullptr;
                }

                poCurvePart = ReadCurveGeometry(
                    poCurveRecord, /* iRecord = */ -1, nRRID, poSRS);
            }
            else
            {
                CPLAssert(nRRNM == RECORD_NAME_COMPOSITE_CURVE);

                const auto poCompositeCurveRecord =
                    m_oCompositeCurveRecordIndex.FindRecord(nRRID);
                if (!poCompositeCurveRecord)
                {
                    CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(CPLSPrintf(
                        "%s: Value (RRNM=%d, RRID=%d) "
                        "of instance %d of CUCO field does not point to an "
                        "existing record.",
                        GetErrorContext(), static_cast<int>(nRRNM), nRRID,
                        iPart)));
                    return nullptr;
                }

                poCurvePart = ReadCompositeCurveGeometryInternal(
                    poCompositeCurveRecord, /* iRecord = */ -1, nRRID, poSRS,
                    oSetAlreadyVisitedCompositeCurveRecords);
            }

            if (!poCurvePart)
            {
                return nullptr;
            }

            const int nPoints = poLS->getNumPoints();
            const int nPointsPart = poCurvePart->getNumPoints();
            if (nPointsPart == 0)
            {
                CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(CPLSPrintf(
                    "%s: Value (RRNM=%d, RRID=%d) "
                    "%d instance of CUCO field points to an empty curve.",
                    GetErrorContext(), static_cast<int>(nRRNM), nRRID, iPart)));
                return nullptr;
            }
            if (nPoints == 0)
            {
                if (bReverse)
                    poLS->addSubLineString(poCurvePart.get(), nPointsPart - 1,
                                           0);
                else
                    poLS->addSubLineString(poCurvePart.get());
            }
            else
            {
                bool bEndPointMismatch = false;
                if (bReverse)
                {
                    if (poLS->getX(nPoints - 1) ==
                            poCurvePart->getX(nPointsPart - 1) &&
                        poLS->getY(nPoints - 1) ==
                            poCurvePart->getY(nPointsPart - 1))
                    {
                        poLS->addSubLineString(poCurvePart.get(),
                                               nPointsPart - 2, 0);
                    }
                    else
                    {
                        bEndPointMismatch = true;
                    }
                }
                else
                {
                    if (poLS->getX(nPoints - 1) == poCurvePart->getX(0) &&
                        poLS->getY(nPoints - 1) == poCurvePart->getY(0))
                    {
                        poLS->addSubLineString(poCurvePart.get(), 1);
                    }
                    else
                    {
                        bEndPointMismatch = true;
                    }
                }
                if (bEndPointMismatch)
                {
                    CPL_IGNORE_RET_VAL(EMIT_ERROR_OR_WARNING(CPLSPrintf(
                        "%s: Value (RRNM=%d, "
                        "RRID=%d) "
                        "of curve instance %d extremity does not match "
                        "composite curve extremity.",
                        GetErrorContext(), static_cast<int>(nRRNM), nRRID,
                        iPart)));
                    return nullptr;
                }
            }
        }
    }

    poLS->assignSpatialReference(poSRS);
    return poLS;
}

std::unique_ptr<OGRLineString> OGRS101Reader::ReadCompositeCurveGeometry(
    const DDFRecord *poRecord, int iRecord, int nRecordID,
    const OGRSpatialReference *poSRS) const
{
    std::set<int> oSetAlreadyVisitedCompositeCurveRecords;
    return ReadCompositeCurveGeometryInternal(
        poRecord, iRecord, nRecordID, poSRS,
        oSetAlreadyVisitedCompositeCurveRecords);
}

/************************************************************************/
/*                     FillFeatureCompositeCurve()                      */
/************************************************************************/

/** Fill the content of the provided feature from the identified record
 * (of m_oCompositeCurveRecordIndex).
 */
bool OGRS101Reader::FillFeatureCompositeCurve(const DDFRecordIndex &oIndex,
                                              int iRecord,
                                              OGRFeature &oFeature) const
{
    const auto poRecord = oIndex.GetByIndex(iRecord);
    CPLAssert(poRecord);

    const OGRSpatialReference *poSRS =
        oFeature.GetDefnRef()->GetGeomFieldDefn(0)->GetSpatialRef();
    auto poCurve = ReadCompositeCurveGeometry(poRecord, iRecord,
                                              /* nRecordID = */ -1, poSRS);
    if (!poCurve)
    {
        if (m_bStrict)
            return false;  // error message already emitted
    }
    else
    {
        oFeature.SetGeometry(std::move(poCurve));
    }

    return FillFeatureAttributes(oIndex, iRecord, INAS_FIELD, oFeature) &&
           FillFeatureWithNonAttrAssocSubfields(poRecord, iRecord, INAS_FIELD,
                                                oFeature);
}

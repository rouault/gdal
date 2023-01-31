/******************************************************************************
 *
 * Project:  OGR
 * Purpose:  WKB geometry related methods
 * Author:   Even Rouault <even dot rouault at spatialys.com>
 *
 ******************************************************************************
 * Copyright (c) 2022, Even Rouault <even dot rouault at spatialys.com>
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

#include "cpl_error.h"
#include "ogr_wkb.h"
#include "ogr_core.h"
#include "ogr_p.h"

#include <cmath>
#include <climits>
#include <functional>

/************************************************************************/
/*                          OGRWKBNeedSwap()                            */
/************************************************************************/

static inline bool OGRWKBNeedSwap(GByte b)
{
#if CPL_IS_LSB
    const bool bNeedSwap = b == 0;
#else
    const bool bNeedSwap = b == 1;
#endif
    return bNeedSwap;
}

/************************************************************************/
/*                        OGRWKBReadUInt32()                            */
/************************************************************************/

static inline uint32_t OGRWKBReadUInt32(const GByte *pabyWkb, bool bNeedSwap)
{
    uint32_t nVal;
    memcpy(&nVal, pabyWkb, sizeof(nVal));
    if (bNeedSwap)
        CPL_SWAP32PTR(&nVal);
    return nVal;
}

/************************************************************************/
/*                        OGRWKBReadFloat64()                           */
/************************************************************************/

static inline double OGRWKBReadFloat64(const GByte *pabyWkb, bool bNeedSwap)
{
    double dfVal;
    memcpy(&dfVal, pabyWkb, sizeof(dfVal));
    if (bNeedSwap)
        CPL_SWAP64PTR(&dfVal);
    return dfVal;
}

/************************************************************************/
/*                        OGRWKBRingGetArea()                           */
/************************************************************************/

static bool OGRWKBRingGetArea(const GByte *&pabyWkb, size_t &nWKBSize, int nDim,
                              bool bNeedSwap, double &dfArea)
{
    const uint32_t nPoints = OGRWKBReadUInt32(pabyWkb, bNeedSwap);
    if (nPoints >= 4 &&
        (nWKBSize - sizeof(uint32_t)) / (nDim * sizeof(double)) >= nPoints)
    {
        nWKBSize -= sizeof(uint32_t) + nDim * sizeof(double);
        pabyWkb += sizeof(uint32_t);
        // Computation according to Green's Theorem
        // Cf OGRSimpleCurve::get_LinearArea()
        double x_m1 = OGRWKBReadFloat64(pabyWkb, bNeedSwap);
        double y_m1 = OGRWKBReadFloat64(pabyWkb + sizeof(double), bNeedSwap);
        double y_m2 = y_m1;
        dfArea = 0;
        pabyWkb += nDim * sizeof(double);
        for (uint32_t i = 1; i < nPoints; ++i)
        {
            const double x = OGRWKBReadFloat64(pabyWkb, bNeedSwap);
            const double y =
                OGRWKBReadFloat64(pabyWkb + sizeof(double), bNeedSwap);
            pabyWkb += nDim * sizeof(double);
            dfArea += x_m1 * (y - y_m2);
            y_m2 = y_m1;
            x_m1 = x;
            y_m1 = y;
        }
        dfArea += x_m1 * (y_m1 - y_m2);
        dfArea = 0.5 * std::fabs(dfArea);
        return true;
    }
    return false;
}

/************************************************************************/
/*                         OGRWKBGetGeomType()                          */
/************************************************************************/

bool OGRWKBGetGeomType(const GByte *pabyWkb, size_t nWKBSize, bool &bNeedSwap,
                       uint32_t &nType)
{
    if (nWKBSize >= 5)
    {
        bNeedSwap = OGRWKBNeedSwap(pabyWkb[0]);
        nType = OGRWKBReadUInt32(pabyWkb + 1, bNeedSwap);
        return true;
    }
    return false;
}

/************************************************************************/
/*                           OGRWKBIsEmpty()                            */
/************************************************************************/

bool OGRWKBIsEmpty(const GByte *pabyWkb, size_t nWKBSize)
{
    bool bNeedSwap;
    uint32_t nType;
    if (nWKBSize < 5 || !OGRWKBGetGeomType(pabyWkb, nWKBSize, bNeedSwap, nType))
        return true;

    if (nWKBSize == wkbPoint || nWKBSize == wkbPoint25D ||
        nWKBSize == wkbPoint + 1000 ||  // wkbPointZ
        nWKBSize == wkbPointM || nWKBSize == wkbPointZM)
    {
        if (nWKBSize < 5 + 2 * sizeof(double))
            return true;
        const double x = OGRWKBReadFloat64(pabyWkb + 5, bNeedSwap);
        const double y =
            OGRWKBReadFloat64(pabyWkb + 5 + sizeof(double), bNeedSwap);
        return std::isnan(x) && std::isnan(y);
    }
    if (nWKBSize < 9)
        return true;
    const auto nPointsOrParts = OGRWKBReadUInt32(pabyWkb + 5, bNeedSwap);
    return nPointsOrParts == 0;
}

/************************************************************************/
/*                      OGRWKBVisitPoints()                             */
/************************************************************************/

static bool
OGRWKBVisitPoints(const GByte *&pabyWkb, size_t &nWKBSize,
                  const std::function<void(const OGRPoint &)> &visitPoint)
{
    bool bNeedSwap;
    uint32_t nType;
    if (nWKBSize < 5 || !OGRWKBGetGeomType(pabyWkb, nWKBSize, bNeedSwap, nType))
        return false;

    OGRwkbGeometryType eGeomType = wkbUnknown;
    OGRReadWKBGeometryType(pabyWkb, wkbVariantIso, &eGeomType);
    if (eGeomType == wkbUnknown)
        return false;

    const int nDim =
        2 + (wkbHasZ(eGeomType) ? 1 : 0) + (wkbHasM(eGeomType) ? 1 : 0);
    if (wkbFlatten(eGeomType) == wkbPoint)
    {
        if (nWKBSize < 5 + 2 * sizeof(double))
            return false;
        const double x = OGRWKBReadFloat64(pabyWkb + 5, bNeedSwap);
        const double y =
            OGRWKBReadFloat64(pabyWkb + 5 + sizeof(double), bNeedSwap);
        OGRPoint p(x, y);
        visitPoint(p);
        pabyWkb += 5 + nDim * sizeof(double);
        nWKBSize -= 5 + nDim * sizeof(double);
        return true;
    }

    if (nWKBSize < 9)
        return false;
    const auto nPointsOrParts = OGRWKBReadUInt32(pabyWkb + 5, bNeedSwap);
    switch (wkbFlatten(eGeomType))
    {
        case wkbLineString:
        case wkbCircularString:
        {
            pabyWkb += 9;
            nWKBSize -= 9;
            OGRPoint p;
            if (nWKBSize / (nDim * sizeof(double)) < nPointsOrParts)
                return false;
            for (uint32_t i = 0; i < nPointsOrParts; ++i)
            {
                const double x = OGRWKBReadFloat64(
                    pabyWkb + i * nDim * sizeof(double), bNeedSwap);
                const double y = OGRWKBReadFloat64(
                    pabyWkb + sizeof(double) + i * nDim * sizeof(double),
                    bNeedSwap);
                p.setX(x);
                p.setY(y);
                visitPoint(p);
            }
            pabyWkb += nDim * nPointsOrParts * sizeof(double);
            nWKBSize -= nDim * nPointsOrParts * sizeof(double);
            break;
        }

        case wkbPolygon:
        {
            pabyWkb += 9;
            nWKBSize -= 9;
            OGRPoint p;
            for (uint32_t i = 0; i < nPointsOrParts; ++i)
            {
                if (nWKBSize < sizeof(uint32_t))
                    return false;
                const auto nRingPoints = OGRWKBReadUInt32(pabyWkb, bNeedSwap);
                pabyWkb += sizeof(uint32_t);
                nWKBSize -= sizeof(uint32_t);
                if (nWKBSize / (nDim * sizeof(double)) < nRingPoints)
                    return false;
                for (uint32_t j = 0; j < nRingPoints; ++j)
                {
                    const double x = OGRWKBReadFloat64(
                        pabyWkb + j * nDim * sizeof(double), bNeedSwap);
                    const double y = OGRWKBReadFloat64(
                        pabyWkb + sizeof(double) + j * nDim * sizeof(double),
                        bNeedSwap);
                    p.setX(x);
                    p.setY(y);
                    visitPoint(p);
                }
                pabyWkb += nDim * nRingPoints * sizeof(double);
                nWKBSize -= nDim * nRingPoints * sizeof(double);
            }
            break;
        }

        case wkbMultiPoint:
        case wkbMultiLineString:
        case wkbMultiPolygon:
        case wkbGeometryCollection:
        case wkbMultiCurve:
        case wkbMultiSurface:
        case wkbCompoundCurve:
        case wkbCurvePolygon:
        case wkbPolyhedralSurface:
        case wkbTIN:
        {
            pabyWkb += 9;
            nWKBSize -= 9;
            for (uint32_t i = 0; i < nPointsOrParts; ++i)
            {
                if (!OGRWKBVisitPoints(pabyWkb, nWKBSize, visitPoint))
                {
                    return false;
                }
            }
            break;
        }

        default:
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "OGRWKBVisitPoints(): unhandled geometry type %d",
                     eGeomType);
            return false;
        }
    }

    return true;
}

/************************************************************************/
/*                       OGRWKBGetEnvelope()                            */
/************************************************************************/

bool OGRWKBGetEnvelope(const GByte *pabyWkb, size_t nWKBSize,
                       OGREnvelope &sEnvelope)
{
    return OGRWKBVisitPoints(pabyWkb, nWKBSize,
                             [&sEnvelope](const OGRPoint &point)
                             { sEnvelope.Merge(point.getX(), point.getY()); });
}

/************************************************************************/
/*                        OGRWKBPolygonGetArea()                        */
/************************************************************************/

bool OGRWKBPolygonGetArea(const GByte *&pabyWkb, size_t &nWKBSize,
                          double &dfArea)
{
    bool bNeedSwap;
    uint32_t nType;
    if (nWKBSize < 9 || !OGRWKBGetGeomType(pabyWkb, nWKBSize, bNeedSwap, nType))
        return false;

    int nDims = 2;
    if (nType == wkbPolygon)
    {
        // do nothing
    }
    else if (nType == wkbPolygon + 1000 ||  // wkbPolygonZ
             nType == wkbPolygon25D || nType == wkbPolygonM)
    {
        nDims = 3;
    }
    else if (nType == wkbPolygonZM)
    {
        nDims = 4;
    }
    else
    {
        return false;
    }

    const uint32_t nRings = OGRWKBReadUInt32(pabyWkb + 5, bNeedSwap);
    if ((nWKBSize - 9) / sizeof(uint32_t) >= nRings)
    {
        pabyWkb += 9;
        nWKBSize -= 9;
        dfArea = 0;
        if (nRings > 0)
        {
            if (!OGRWKBRingGetArea(pabyWkb, nWKBSize, nDims, bNeedSwap, dfArea))
                return false;
            for (uint32_t i = 1; i < nRings; ++i)
            {
                double dfRingArea;
                if (!OGRWKBRingGetArea(pabyWkb, nWKBSize, nDims, bNeedSwap,
                                       dfRingArea))
                    return false;
                dfArea -= dfRingArea;
            }
        }
        return true;
    }
    return false;
}

/************************************************************************/
/*                    OGRWKBMultiPolygonGetArea()                       */
/************************************************************************/

bool OGRWKBMultiPolygonGetArea(const GByte *&pabyWkb, size_t &nWKBSize,
                               double &dfArea)
{
    if (nWKBSize < 9)
        return false;

    const bool bNeedSwap = OGRWKBNeedSwap(pabyWkb[0]);
    const uint32_t nPolys = OGRWKBReadUInt32(pabyWkb + 5, bNeedSwap);
    if ((nWKBSize - 9) / 9 >= nPolys)
    {
        pabyWkb += 9;
        nWKBSize -= 9;
        dfArea = 0;
        for (uint32_t i = 0; i < nPolys; ++i)
        {
            double dfPolyArea;
            if (!OGRWKBPolygonGetArea(pabyWkb, nWKBSize, dfPolyArea))
                return false;
            dfArea += dfPolyArea;
        }
        return true;
    }
    return false;
}

/************************************************************************/
/*                            WKBFromEWKB()                             */
/************************************************************************/

const GByte *WKBFromEWKB(GByte *pabyEWKB, size_t nEWKBSize, size_t &nWKBSizeOut,
                         int *pnSRIDOut)
{
    if (nEWKBSize < 5U)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Invalid EWKB content : %u bytes",
                 static_cast<unsigned>(nEWKBSize));
        return nullptr;
    }

    const GByte *pabyWKB = pabyEWKB;

    /* -------------------------------------------------------------------- */
    /*      PostGIS EWKB format includes an SRID, but this won't be         */
    /*      understood by OGR, so if the SRID flag is set, we remove the    */
    /*      SRID (bytes at offset 5 to 8).                                  */
    /* -------------------------------------------------------------------- */
    if (nEWKBSize > 9 &&
        ((pabyEWKB[0] == 0 /* big endian */ && (pabyEWKB[1] & 0x20)) ||
         (pabyEWKB[0] != 0 /* little endian */ && (pabyEWKB[4] & 0x20))))
    {
        if (pnSRIDOut)
        {
            memcpy(pnSRIDOut, pabyEWKB + 5, 4);
            const OGRwkbByteOrder eByteOrder =
                (pabyEWKB[0] == 0 ? wkbXDR : wkbNDR);
            if (OGR_SWAP(eByteOrder))
                *pnSRIDOut = CPL_SWAP32(*pnSRIDOut);
        }

        // Drop the SRID flag
        if (pabyEWKB[0] == 0)
            pabyEWKB[1] &= (~0x20);
        else
            pabyEWKB[4] &= (~0x20);

        // Move 5 first bytes of EWKB 4 bytes later to create regular WKB
        memmove(pabyEWKB + 4, pabyEWKB, 5);
        memset(pabyEWKB, 0, 4);
        // and make pabyWKB point to that
        pabyWKB += 4;
        nWKBSizeOut = nEWKBSize - 4;
    }
    else
    {
        if (pnSRIDOut)
        {
            *pnSRIDOut = INT_MIN;
        }
        nWKBSizeOut = nEWKBSize;
    }

    return pabyWKB;
}

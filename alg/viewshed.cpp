/******************************************************************************
 *
 * Project:  Viewshed Generation
 * Purpose:  Core algorithm implementation for viewshed generation.
 * Author:   Tamas Szekeres, szekerest@gmail.com
 *
 ******************************************************************************
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

#include "gdal_alg.h"
#include "gdal_priv.h"

#include "viewshed.h"

/************************************************************************/
/*                        GDALViewshedGenerate()                        */
/************************************************************************/

GDALDatasetH GDALViewshedGenerate(
    GDALRasterBandH hBand, const char *pszDriverName,
    const char *pszTargetRasterName, CSLConstList papszCreationOptions,
    double dfObserverX, double dfObserverY, double dfObserverHeight,
    double dfTargetHeight, double dfVisibleVal, double dfInvisibleVal,
    double dfOutOfRangeVal, double dfNoDataVal, double dfCurvCoeff,
    GDALViewshedMode eMode, double dfMaxDistance, GDALProgressFunc pfnProgress,
    [[maybe_unused]] void *pProgressArg, GDALViewshedOutputType heightMode,
    [[maybe_unused]] CSLConstList papszExtraOptions)
{
    using namespace gdal;

    Viewshed::Options oOpts;
    oOpts.outputFormat = pszDriverName;
    oOpts.outputFilename = pszTargetRasterName;
    oOpts.creationOpts = papszCreationOptions;
    oOpts.observer.x = dfObserverX;
    oOpts.observer.y = dfObserverY;
    oOpts.observer.z = dfObserverHeight;
    oOpts.targetHeight = dfTargetHeight;
    oOpts.curveCoeff = dfCurvCoeff;
    oOpts.maxDistance = dfMaxDistance;
    oOpts.nodataVal = dfNoDataVal;

    if (eMode == GVM_Edge)
        oOpts.cellMode = Viewshed::CellMode::Edge;
    else if (eMode == GVM_Diagonal)
        oOpts.cellMode = Viewshed::CellMode::Diagonal;
    if (eMode == GVM_Min)
        oOpts.cellMode = Viewshed::CellMode::Min;
    if (eMode == GVM_Max)
        oOpts.cellMode = Viewshed::CellMode::Max;

    if (heightMode == GVOT_MIN_TARGET_HEIGHT_FROM_DEM)
        oOpts.outputMode = Viewshed::OutputMode::DEM;
    else if (heightMode == GVOT_MIN_TARGET_HEIGHT_FROM_GROUND)
        oOpts.outputMode = Viewshed::OutputMode::Ground;
    else
        oOpts.outputMode = Viewshed::OutputMode::Normal;

    //ABELL - Perhaps assert these for range.
    oOpts.visibleVal = static_cast<uint8_t>(dfVisibleVal);
    oOpts.invisibleVal = static_cast<uint8_t>(dfInvisibleVal);
    oOpts.outOfRangeVal = static_cast<uint8_t>(dfOutOfRangeVal);

    gdal::Viewshed v(oOpts);

    //ABELL - Make a function for progress that captures the progress argument.
    v.run(hBand, pfnProgress);

    return GDALDataset::FromHandle(v.output().release());
}


namespace gdal
{

namespace
{

static void SetVisibility(int iPixel, double dfZ, double dfZTarget,
                                 double *padfZVal, std::vector<GByte> &vResult,
                                 GByte byVisibleVal, GByte byInvisibleVal)
{
    if (padfZVal[iPixel] + dfZTarget < dfZ)
        vResult[iPixel] = byInvisibleVal;
    else
        vResult[iPixel] = byVisibleVal;

    if (padfZVal[iPixel] < dfZ)
        padfZVal[iPixel] = dfZ;
}

static bool AdjustHeightInRange(const double *adfGeoTransform,
                                       int iPixel, int iLine, double &dfHeight,
                                       double dfDistance2, double dfCurvCoeff,
                                       double dfSphereDiameter)
{
    if (dfDistance2 <= 0 && dfCurvCoeff == 0)
        return true;

    double dfX = adfGeoTransform[1] * iPixel + adfGeoTransform[2] * iLine;
    double dfY = adfGeoTransform[4] * iPixel + adfGeoTransform[5] * iLine;
    double dfR2 = dfX * dfX + dfY * dfY;

    /* calc adjustment */
    if (dfCurvCoeff != 0 &&
        dfSphereDiameter != std::numeric_limits<double>::infinity())
        dfHeight -= dfCurvCoeff * dfR2 / dfSphereDiameter;

    if (dfDistance2 > 0 && dfR2 > dfDistance2)
        return false;

    return true;
}

static double CalcHeightLine(int i, double Za, double Zo)
{
    if (i == 1)
        return Za;
    else
        return (Za - Zo) / (i - 1) + Za;
}

static double CalcHeightDiagonal(int i, int j, double Za, double Zb,
                                        double Zo)
{
    return ((Za - Zo) * i + (Zb - Zo) * j) / (i + j - 1) + Zo;
}

static double CalcHeightEdge(int i, int j, double Za, double Zb,
                                    double Zo)
{
    if (i == j)
        return CalcHeightLine(i, Za, Zo);
    else
        return ((Za - Zo) * i + (Zb - Zo) * (j - i)) / (j - 1) + Zo;
}

static double CalcHeight(double dfZ, double dfZ2, Viewshed::CellMode eMode)
{
    double dfHeight;

    switch (eMode)
    {
    case Viewshed::CellMode::Edge:
        dfHeight = dfZ2;
        break;
    case Viewshed::CellMode::Max:
        dfHeight = std::max(dfZ, dfZ2);
        break;
    case Viewshed::CellMode::Min:
        dfHeight = std::min(dfZ, dfZ2);
        break;
    case Viewshed::CellMode::Diagonal:
        dfHeight = dfZ;
        break;
    }
    return dfHeight;
}

} // unnamed namespace

bool Viewshed::run(GDALRasterBandH hBand, GDALProgressFunc pfnProgress)
{
    if (!pfnProgress)
        pfnProgress = GDALDummyProgress;

    if (!pfnProgress(0.0, "", nullptr))
    {
        CPLError(CE_Failure, CPLE_UserInterrupt, "User terminated");
        return false;
    }

    /* set up geotransformation */
    std::array<double, 6> adfGeoTransform{{0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
    GDALDatasetH hSrcDS = GDALGetBandDataset(hBand);
    if (hSrcDS != nullptr)
        GDALGetGeoTransform(hSrcDS, adfGeoTransform.data());

    double adfInvGeoTransform[6];
    if (!GDALInvGeoTransform(adfGeoTransform.data(), adfInvGeoTransform))
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Cannot invert geotransform");
        return false;
    }

    /* calculate observer position */
    double dfX, dfY;
    GDALApplyGeoTransform(adfInvGeoTransform, oOpts.observer.x,
        oOpts.observer.y, &dfX, &dfY);
    int nX = static_cast<int>(dfX);
    int nY = static_cast<int>(dfY);

    int nXSize = GDALGetRasterBandXSize(hBand);
    int nYSize = GDALGetRasterBandYSize(hBand);

    if (nX < 0 || nX > nXSize || nY < 0 || nY > nYSize)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "The observer location falls outside of the DEM area");
        return false;
    }

    /* calculate the area of interest */
    constexpr double EPSILON = 1e-8;

    //ABELL - Do we care about overflow?
    int nXStart = 0;
    int nYStart = 0;
    int nXStop = nXSize;
    int nYStop = nYSize;
    if (oOpts.maxDistance > 0)
    {
        nXStart = static_cast<int>(
            std::floor(nX - adfInvGeoTransform[1] * oOpts.maxDistance +
                       EPSILON));
        nXStop = static_cast<int>(
            std::ceil(nX + adfInvGeoTransform[1] * oOpts.maxDistance -
                      EPSILON) + 1);
        nYStart = static_cast<int>(
            std::floor(nY - std::fabs(adfInvGeoTransform[5]) *
                       oOpts.maxDistance + EPSILON)) -
                       (adfInvGeoTransform[5] > 0 ? 1 : 0);
        nYStop = static_cast<int>(
            std::ceil(nY + std::fabs(adfInvGeoTransform[5]) *
                      oOpts.maxDistance - EPSILON) +
                      (adfInvGeoTransform[5] < 0 ? 1 : 0));
    }
    nXStart = std::max(nXStart, 0);
    nYStart = std::max(nYStart, 0);
    nXStop = std::min(nXStop, nXSize);
    nYStop = std::min(nYStop, nYSize);

    /* normalize horizontal index (0 - nXSize) */
    nXSize = nXStop - nXStart;
    nX -= nXStart;

    nYSize = nYStop - nYStart;

    if (nXSize == 0 || nYSize == 0)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Invalid target raster size");
        return false;
    }

    std::vector<double> vFirstLineVal;
    std::vector<double> vLastLineVal;
    std::vector<double> vThisLineVal;
    std::vector<GByte> vResult;
    std::vector<double> vHeightResult;

    try
    {
        vFirstLineVal.resize(nXSize);
        vLastLineVal.resize(nXSize);
        vThisLineVal.resize(nXSize);
        vResult.resize(nXSize);

        if (oOpts.outputMode != OutputMode::Normal)
            vHeightResult.resize(nXSize);
    }
    catch (...)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Cannot allocate vectors for viewshed");
        return false;
    }

    double *padfFirstLineVal = vFirstLineVal.data();
    double *padfLastLineVal = vLastLineVal.data();
    double *padfThisLineVal = vThisLineVal.data();
    GByte *pabyResult = vResult.data();
    double *dfHeightResult = vHeightResult.data();

    GDALDriverManager *hMgr = GetGDALDriverManager();
    GDALDriver *hDriver = hMgr->GetDriverByName(oOpts.outputFormat.c_str());
    if (!hDriver)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Cannot get driver");
        return false;
    }

    /* create output raster */
    poDstDS.reset(
        hDriver->Create(oOpts.outputFilename.c_str(), nXSize, nYSize, 1,
            oOpts.outputMode == OutputMode::Normal ? GDT_Byte : GDT_Float64,
            const_cast<char **>(oOpts.creationOpts.List())));
    if (!poDstDS)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Cannot create dataset for %s",
                 oOpts.outputFilename.c_str());
        return false;
    }
    /* copy srs */
    if (hSrcDS)
        poDstDS->SetSpatialRef(
            GDALDataset::FromHandle(hSrcDS)->GetSpatialRef());

    std::array<double, 6> adfDstGeoTransform;
    adfDstGeoTransform[0] = adfGeoTransform[0] + adfGeoTransform[1] * nXStart +
                            adfGeoTransform[2] * nYStart;
    adfDstGeoTransform[1] = adfGeoTransform[1];
    adfDstGeoTransform[2] = adfGeoTransform[2];
    adfDstGeoTransform[3] = adfGeoTransform[3] + adfGeoTransform[4] * nXStart +
                            adfGeoTransform[5] * nYStart;
    adfDstGeoTransform[4] = adfGeoTransform[4];
    adfDstGeoTransform[5] = adfGeoTransform[5];
    poDstDS->SetGeoTransform(adfDstGeoTransform.data());

    auto hTargetBand = poDstDS->GetRasterBand(1);
    if (hTargetBand == nullptr)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Cannot get band for %s",
                 oOpts.outputFilename.c_str());
        return false;
    }

    if (oOpts.nodataVal >= 0)
        GDALSetRasterNoDataValue( hTargetBand, oOpts.nodataVal);

    /* process first line */
    if (GDALRasterIO(hBand, GF_Read, nXStart, nY, nXSize, 1, padfFirstLineVal,
                     nXSize, 1, GDT_Float64, 0, 0))
    {
        CPLError(
            CE_Failure, CPLE_AppDefined,
            "RasterIO error when reading DEM at position(%d, %d), size(%d, %d)",
            nXStart, nY, nXSize, 1);
        return false;
    }

    const double dfZObserver = oOpts.observer.z + padfFirstLineVal[nX];
    double dfZ = 0.0;
    const double dfDistance2 = oOpts.maxDistance * oOpts.maxDistance;

    /* If we can't get a SemiMajor axis from the SRS, it will be
     * SRS_WGS84_SEMIMAJOR
     */
    double dfSphereDiameter(std::numeric_limits<double>::infinity());
    const OGRSpatialReference *poDstSRS = poDstDS->GetSpatialRef();
    if (poDstSRS)
    {
        OGRErr eSRSerr;
        double dfSemiMajor = poDstSRS->GetSemiMajor(&eSRSerr);

        /* If we fetched the axis from the SRS, use it */
        if (eSRSerr != OGRERR_FAILURE)
            dfSphereDiameter = dfSemiMajor * 2.0;
        else
            CPLDebug("GDALViewshedGenerate",
                     "Unable to fetch SemiMajor axis from spatial reference");
    }

    /* mark the observer point as visible */
    double dfGroundLevel = 0;
    if (oOpts.outputMode == OutputMode::DEM)
        dfGroundLevel = padfFirstLineVal[nX];

    pabyResult[nX] = oOpts.visibleVal;

    //ABELL - Do we care about this conditional?
    if (oOpts.outputMode != OutputMode::Normal)
        dfHeightResult[nX] = dfGroundLevel;

    dfGroundLevel = 0;
    if (nX > 0)
    {
        if (oOpts.outputMode == OutputMode::DEM)
            dfGroundLevel = padfFirstLineVal[nX - 1];
        CPL_IGNORE_RET_VAL(AdjustHeightInRange(
            adfGeoTransform.data(), 1, 0, padfFirstLineVal[nX - 1], dfDistance2,
            oOpts.curveCoeff, dfSphereDiameter));
        pabyResult[nX - 1] = oOpts.visibleVal;
        if (oOpts.outputMode != OutputMode::Normal)
            dfHeightResult[nX - 1] = dfGroundLevel;
    }
    if (nX < nXSize - 1)
    {
        if (oOpts.outputMode == OutputMode::DEM)
        dfGroundLevel = padfFirstLineVal[nX + 1];
        CPL_IGNORE_RET_VAL(AdjustHeightInRange(
            adfGeoTransform.data(), 1, 0, padfFirstLineVal[nX + 1], dfDistance2,
            oOpts.curveCoeff, dfSphereDiameter));
        pabyResult[nX + 1] = oOpts.visibleVal;
        if (oOpts.outputMode != OutputMode::Normal)
            dfHeightResult[nX + 1] = dfGroundLevel;
    }

    /* process left direction */
    for (int iPixel = nX - 2; iPixel >= 0; iPixel--)
    {
        dfGroundLevel = 0;
        if (oOpts.outputMode == OutputMode::DEM)
            dfGroundLevel = padfFirstLineVal[iPixel];

        if (AdjustHeightInRange(adfGeoTransform.data(), nX - iPixel, 0,
            padfFirstLineVal[iPixel], dfDistance2, oOpts.curveCoeff,
            dfSphereDiameter))
        {
            dfZ = CalcHeightLine(nX - iPixel, padfFirstLineVal[iPixel + 1],
                                 dfZObserver);

            if (oOpts.outputMode != OutputMode::Normal)
                dfHeightResult[iPixel] = std::max(
                    0.0, (dfZ - padfFirstLineVal[iPixel] + dfGroundLevel));

            SetVisibility(iPixel, dfZ, oOpts.targetHeight, padfFirstLineVal,
                          vResult, oOpts.visibleVal, oOpts.invisibleVal);
        }
        else
        {
            for (; iPixel >= 0; iPixel--)
            {
                pabyResult[iPixel] = oOpts.outOfRangeVal;
                if (oOpts.outputMode != OutputMode::Normal)
                    dfHeightResult[iPixel] = oOpts.outOfRangeVal;
            }
        }
    }
    /* process right direction */
    for (int iPixel = nX + 2; iPixel < nXSize; iPixel++)
    {
        dfGroundLevel = 0;
        if (oOpts.outputMode == OutputMode::DEM)
            dfGroundLevel = padfFirstLineVal[iPixel];
        if (AdjustHeightInRange(adfGeoTransform.data(), iPixel - nX, 0,
            padfFirstLineVal[iPixel], dfDistance2, oOpts.curveCoeff,
            dfSphereDiameter))
        {
            dfZ = CalcHeightLine(iPixel - nX, padfFirstLineVal[iPixel - 1],
                                 dfZObserver);

            if (oOpts.outputMode != OutputMode::Normal)
                dfHeightResult[iPixel] = std::max(
                    0.0, (dfZ - padfFirstLineVal[iPixel] + dfGroundLevel));

            SetVisibility(iPixel, dfZ, oOpts.targetHeight, padfFirstLineVal,
                          vResult, oOpts.visibleVal, oOpts.invisibleVal);
        }
        else
        {
            for (; iPixel < nXSize; iPixel++)
            {
                pabyResult[iPixel] = oOpts.outOfRangeVal;
                if (oOpts.outputMode != OutputMode::Normal)
                    dfHeightResult[iPixel] = oOpts.outOfRangeVal;
            }
        }
    }
    /* write result line */

    void *data;
    GDALDataType dataType;
    if (oOpts.outputMode == OutputMode::Normal)
    {
        data = static_cast<void *>(pabyResult);
        dataType = GDT_Byte;
    }
    else
    {
        data = static_cast<void *>(dfHeightResult);
        dataType = GDT_Float64;
    }
    if (GDALRasterIO(hTargetBand, GF_Write, 0, nY - nYStart, nXSize, 1,
                     data, nXSize, 1, dataType, 0, 0))
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "RasterIO error when writing target raster at position "
                 "(%d,%d), size (%d,%d)",
                 0, nY - nYStart, nXSize, 1);
        return false;
    }

    /* scan upwards */
    std::copy(vFirstLineVal.begin(), vFirstLineVal.end(), vLastLineVal.begin());
    for (int iLine = nY - 1; iLine >= nYStart; iLine--)
    {
        if (GDALRasterIO(hBand, GF_Read, nXStart, iLine, nXSize, 1,
                         padfThisLineVal, nXSize, 1, GDT_Float64, 0, 0))
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "RasterIO error when reading DEM at position (%d,%d), "
                     "size (%d,%d)",
                     nXStart, iLine, nXSize, 1);
            return false;
        }

        /* set up initial point on the scanline */
        dfGroundLevel = 0;
        if (oOpts.outputMode == OutputMode::DEM)
            dfGroundLevel = padfThisLineVal[nX];
        if (AdjustHeightInRange(adfGeoTransform.data(), 0, nY - iLine,
            padfThisLineVal[nX], dfDistance2, oOpts.curveCoeff,
            dfSphereDiameter))
        {
            dfZ = CalcHeightLine(nY - iLine, padfLastLineVal[nX], dfZObserver);

            if (oOpts.outputMode != OutputMode::Normal)
                dfHeightResult[nX] =
                    std::max(0.0, (dfZ - padfThisLineVal[nX] + dfGroundLevel));

            SetVisibility(nX, dfZ, oOpts.targetHeight, padfThisLineVal,
                vResult, oOpts.visibleVal, oOpts.invisibleVal);
        }
        else
        {
            pabyResult[nX] = oOpts.outOfRangeVal;
            if (oOpts.outputMode != OutputMode::Normal)
                dfHeightResult[nX] = oOpts.outOfRangeVal;
        }

        /* process left direction */
        for (int iPixel = nX - 1; iPixel >= 0; iPixel--)
        {
            dfGroundLevel = 0;
            if (oOpts.outputMode == OutputMode::DEM)
                dfGroundLevel = padfThisLineVal[iPixel];
            if (AdjustHeightInRange(adfGeoTransform.data(), nX - iPixel,
                nY - iLine, padfThisLineVal[iPixel],
                dfDistance2, oOpts.curveCoeff, dfSphereDiameter))
            {
                if (oOpts.cellMode != CellMode::Edge)
                    dfZ = CalcHeightDiagonal(
                        nX - iPixel, nY - iLine, padfThisLineVal[iPixel + 1],
                        padfLastLineVal[iPixel], dfZObserver);

                if (oOpts.cellMode != CellMode::Diagonal)
                {
                    double dfZ2 =
                        nX - iPixel >= nY - iLine
                            ? CalcHeightEdge(nY - iLine, nX - iPixel,
                                             padfLastLineVal[iPixel + 1],
                                             padfThisLineVal[iPixel + 1],
                                             dfZObserver)
                            : CalcHeightEdge(nX - iPixel, nY - iLine,
                                             padfLastLineVal[iPixel + 1],
                                             padfLastLineVal[iPixel],
                                             dfZObserver);
                    dfZ = CalcHeight(dfZ, dfZ2, oOpts.cellMode);
                }

                if (oOpts.outputMode != OutputMode::Normal)
                    dfHeightResult[iPixel] = std::max(
                        0.0, (dfZ - padfThisLineVal[iPixel] + dfGroundLevel));

                SetVisibility(iPixel, dfZ, oOpts.targetHeight, padfThisLineVal,
                              vResult, oOpts.visibleVal, oOpts.invisibleVal);
            }
            else
            {
                for (; iPixel >= 0; iPixel--)
                {
                    pabyResult[iPixel] = oOpts.outOfRangeVal;
                    if (oOpts.outputMode != OutputMode::Normal)
                        dfHeightResult[iPixel] = oOpts.outOfRangeVal;
                }
            }
        }
        /* process right direction */
        for (int iPixel = nX + 1; iPixel < nXSize; iPixel++)
        {
            dfGroundLevel = 0;
            if (oOpts.outputMode == OutputMode::DEM)
                dfGroundLevel = padfThisLineVal[iPixel];

            if (AdjustHeightInRange(adfGeoTransform.data(), iPixel - nX,
                nY - iLine, padfThisLineVal[iPixel], dfDistance2,
                oOpts.curveCoeff, dfSphereDiameter))
            {
                if (oOpts.cellMode != CellMode::Edge)
                    dfZ = CalcHeightDiagonal(
                        iPixel - nX, nY - iLine, padfThisLineVal[iPixel - 1],
                        padfLastLineVal[iPixel], dfZObserver);
                if (oOpts.cellMode != CellMode::Diagonal)
                {
                    double dfZ2 =
                        iPixel - nX >= nY - iLine
                            ? CalcHeightEdge(nY - iLine, iPixel - nX,
                                             padfLastLineVal[iPixel - 1],
                                             padfThisLineVal[iPixel - 1],
                                             dfZObserver)
                            : CalcHeightEdge(iPixel - nX, nY - iLine,
                                             padfLastLineVal[iPixel - 1],
                                             padfLastLineVal[iPixel],
                                             dfZObserver);
                    dfZ = CalcHeight(dfZ, dfZ2, oOpts.cellMode);
                }

                if (oOpts.outputMode != OutputMode::Normal)
                    dfHeightResult[iPixel] = std::max(
                        0.0, (dfZ - padfThisLineVal[iPixel] + dfGroundLevel));

                SetVisibility(iPixel, dfZ, oOpts.targetHeight, padfThisLineVal,
                              vResult, oOpts.visibleVal, oOpts.invisibleVal);
            }
            else
            {
                for (; iPixel < nXSize; iPixel++)
                {
                    pabyResult[iPixel] = oOpts.outOfRangeVal;
                    if (oOpts.outputMode != OutputMode::Normal)
                        dfHeightResult[iPixel] = oOpts.outOfRangeVal;
                }
            }
        }

        /* write result line */
        if (GDALRasterIO(
                hTargetBand, GF_Write, 0, iLine - nYStart, nXSize, 1,
                data, nXSize, 1, dataType, 0, 0))
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "RasterIO error when writing target raster at position "
                     "(%d,%d), size (%d,%d)",
                     0, iLine - nYStart, nXSize, 1);
            return false;
        }

        std::swap(padfLastLineVal, padfThisLineVal);

        if (!pfnProgress((nY - iLine) / static_cast<double>(nYSize), "",
                         nullptr))
        {
            CPLError(CE_Failure, CPLE_UserInterrupt, "User terminated");
            return false;
        }
    }

    /* scan downwards */
    memcpy(padfLastLineVal, padfFirstLineVal, nXSize * sizeof(double));
    for (int iLine = nY + 1; iLine < nYStop; iLine++)
    {
        if (GDALRasterIO(hBand, GF_Read, nXStart, iLine, nXSize, 1,
                         padfThisLineVal, nXSize, 1, GDT_Float64, 0,
                         0))
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "RasterIO error when reading DEM at position (%d,%d), "
                     "size (%d,%d)",
                     nXStart, iLine, nXStop - nXStart, 1);
            return false;
        }

        /* set up initial point on the scanline */
        dfGroundLevel = 0;
        if (oOpts.outputMode == OutputMode::DEM)
            dfGroundLevel = padfThisLineVal[nX];

        if (AdjustHeightInRange(adfGeoTransform.data(), 0, iLine - nY,
            padfThisLineVal[nX], dfDistance2, oOpts.curveCoeff,
            dfSphereDiameter))
        {
            dfZ = CalcHeightLine(iLine - nY, padfLastLineVal[nX], dfZObserver);

            if (oOpts.outputMode != OutputMode::Normal)
                dfHeightResult[nX] =
                    std::max(0.0, (dfZ - padfThisLineVal[nX] + dfGroundLevel));

            SetVisibility(nX, dfZ, oOpts.targetHeight, padfThisLineVal, vResult,
                          oOpts.visibleVal, oOpts.invisibleVal);
        }
        else
        {
            pabyResult[nX] = oOpts.outOfRangeVal;
            if (oOpts.outputMode != OutputMode::Normal)
                dfHeightResult[nX] = oOpts.outOfRangeVal;
        }

        /* process left direction */
        for (int iPixel = nX - 1; iPixel >= 0; iPixel--)
        {
            dfGroundLevel = 0;
            if (oOpts.outputMode == OutputMode::DEM)
                dfGroundLevel = padfThisLineVal[iPixel];

            if (AdjustHeightInRange(adfGeoTransform.data(), nX - iPixel,
                iLine - nY, padfThisLineVal[iPixel], dfDistance2,
                oOpts.curveCoeff, dfSphereDiameter))
            {
                if (oOpts.cellMode != CellMode::Edge)
                    dfZ = CalcHeightDiagonal(
                        nX - iPixel, iLine - nY, padfThisLineVal[iPixel + 1],
                        padfLastLineVal[iPixel], dfZObserver);

                if (oOpts.cellMode != CellMode::Diagonal)
                {
                    double dfZ2 =
                        nX - iPixel >= iLine - nY
                            ? CalcHeightEdge(iLine - nY, nX - iPixel,
                                             padfLastLineVal[iPixel + 1],
                                             padfThisLineVal[iPixel + 1],
                                             dfZObserver)
                            : CalcHeightEdge(nX - iPixel, iLine - nY,
                                             padfLastLineVal[iPixel + 1],
                                             padfLastLineVal[iPixel],
                                             dfZObserver);
                    dfZ = CalcHeight(dfZ, dfZ2, oOpts.cellMode);
                }

                if (oOpts.outputMode != OutputMode::Normal)
                    dfHeightResult[iPixel] = std::max(
                        0.0, (dfZ - padfThisLineVal[iPixel] + dfGroundLevel));

                SetVisibility(iPixel, dfZ, oOpts.targetHeight, padfThisLineVal,
                              vResult, oOpts.visibleVal, oOpts.invisibleVal);
            }
            else
            {
                for (; iPixel >= 0; iPixel--)
                {
                    pabyResult[iPixel] = oOpts.outOfRangeVal;
                    if (oOpts.outputMode != OutputMode::Normal)
                        dfHeightResult[iPixel] = oOpts.outOfRangeVal;
                }
            }
        }
        /* process right direction */
        for (int iPixel = nX + 1; iPixel < nXSize; iPixel++)
        {
            dfGroundLevel = 0;
            if (oOpts.outputMode == OutputMode::DEM)
                dfGroundLevel = padfThisLineVal[iPixel];

            if (AdjustHeightInRange(adfGeoTransform.data(), iPixel - nX,
               iLine - nY, padfThisLineVal[iPixel],
               dfDistance2, oOpts.curveCoeff, dfSphereDiameter))
            {
                if (oOpts.cellMode != CellMode::Edge)
                    dfZ = CalcHeightDiagonal(
                        iPixel - nX, iLine - nY, padfThisLineVal[iPixel - 1],
                        padfLastLineVal[iPixel], dfZObserver);

                if (oOpts.cellMode != CellMode::Diagonal)
                {
                    double dfZ2 =
                        iPixel - nX >= iLine - nY
                            ? CalcHeightEdge(iLine - nY, iPixel - nX,
                                             padfLastLineVal[iPixel - 1],
                                             padfThisLineVal[iPixel - 1],
                                             dfZObserver)
                            : CalcHeightEdge(iPixel - nX, iLine - nY,
                                             padfLastLineVal[iPixel - 1],
                                             padfLastLineVal[iPixel],
                                             dfZObserver);
                    dfZ = CalcHeight(dfZ, dfZ2, oOpts.cellMode);
                }

                if (oOpts.outputMode != OutputMode::Normal)
                    dfHeightResult[iPixel] = std::max(
                        0.0, (dfZ - padfThisLineVal[iPixel] + dfGroundLevel));

                SetVisibility(iPixel, dfZ, oOpts.targetHeight, padfThisLineVal,
                              vResult, oOpts.visibleVal, oOpts.invisibleVal);
            }
            else
            {
                for (; iPixel < nXSize; iPixel++)
                {
                    pabyResult[iPixel] = oOpts.outOfRangeVal;
                    if (oOpts.outputMode != OutputMode::Normal)
                        dfHeightResult[iPixel] = oOpts.outOfRangeVal;
                }
            }
        }

        /* write result line */
        if (GDALRasterIO(
                hTargetBand, GF_Write, 0, iLine - nYStart, nXSize, 1,
                data, nXSize, 1, dataType, 0, 0))
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "RasterIO error when writing target raster at position "
                     "(%d,%d), size (%d,%d)",
                     0, iLine - nYStart, nXSize, 1);
            return false;
        }

        std::swap(padfLastLineVal, padfThisLineVal);

        if (!pfnProgress((iLine - nYStart) / static_cast<double>(nYSize), "",
                         nullptr))
        {
            CPLError(CE_Failure, CPLE_UserInterrupt, "User terminated");
            return false;
        }
    }

    if (!pfnProgress(1.0, "", nullptr))
    {
        CPLError(CE_Failure, CPLE_UserInterrupt, "User terminated");
        return false;
    }

    return true;
}

} // namespace gdal

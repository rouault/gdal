/******************************************************************************
 *
 * Project:  CPL - Common Portability Library
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 * Purpose:  Progress function implementations.
 *
 ******************************************************************************
 * Copyright (c) 2013, Frank Warmerdam
 *
 * SPDX-License-Identifier: MIT
 ****************************************************************************/

#include "cpl_progress.h"

#include <cmath>
#include <cstdio>
#include <ctime>

#include <algorithm>

#ifdef _WIN32
#include <io.h>  // isatty
#else
#include <unistd.h>  // isatty
#endif

#include "cpl_conv.h"

/************************************************************************/
/*                         GDALDummyProgress()                          */
/************************************************************************/

/**
 * \brief Stub progress function.
 *
 * This is a stub (does nothing) implementation of the GDALProgressFunc()
 * semantics.  It is primarily useful for passing to functions that take
 * a GDALProgressFunc() argument but for which the application does not want
 * to use one of the other progress functions that actually do something.
 */

int CPL_STDCALL GDALDummyProgress(double /* dfComplete */,
                                  const char * /* pszMessage */,
                                  void * /* pData */)
{
    return TRUE;
}

/************************************************************************/
/*                         GDALScaledProgress()                         */
/************************************************************************/
typedef struct
{
    GDALProgressFunc pfnProgress;
    void *pData;
    double dfMin;
    double dfMax;
} GDALScaledProgressInfo;

/**
 * \brief Scaled progress transformer.
 *
 * This is the progress function that should be passed along with the
 * callback data returned by GDALCreateScaledProgress().
 */

int CPL_STDCALL GDALScaledProgress(double dfComplete, const char *pszMessage,
                                   void *pData)

{
    GDALScaledProgressInfo *psInfo =
        reinterpret_cast<GDALScaledProgressInfo *>(pData);

    // Optimization if GDALCreateScaledProgress() provided with
    // GDALDummyProgress.
    if (psInfo == nullptr)
        return TRUE;

    return psInfo->pfnProgress(dfComplete * (psInfo->dfMax - psInfo->dfMin) +
                                   psInfo->dfMin,
                               pszMessage, psInfo->pData);
}

/************************************************************************/
/*                      GDALCreateScaledProgress()                      */
/************************************************************************/

/**
 * \brief Create scaled progress transformer.
 *
 * Sometimes when an operations wants to report progress it actually
 * invokes several subprocesses which also take GDALProgressFunc()s,
 * and it is desirable to map the progress of each sub operation into
 * a portion of 0.0 to 1.0 progress of the overall process.  The scaled
 * progress function can be used for this.
 *
 * For each subsection a scaled progress function is created and
 * instead of passing the overall progress func down to the sub functions,
 * the GDALScaledProgress() function is passed instead.
 *
 * @param dfMin the value to which 0.0 in the sub operation is mapped.
 * @param dfMax the value to which 1.0 is the sub operation is mapped.
 * @param pfnProgress the overall progress function.
 * @param pData the overall progress function callback data.
 *
 * @return pointer to pass as pProgressArg to sub functions.  Should be freed
 * with GDALDestroyScaledProgress().
 *
 * Example:
 *
 * \code
 *   int MyOperation( ..., GDALProgressFunc pfnProgress, void *pProgressData );
 *
 *   {
 *       void *pScaledProgress;
 *
 *       pScaledProgress = GDALCreateScaledProgress( 0.0, 0.5, pfnProgress,
 *                                                   pProgressData );
 *       GDALDoLongSlowOperation( ..., GDALScaledProgress, pScaledProgress );
 *       GDALDestroyScaledProgress( pScaledProgress );
 *
 *       pScaledProgress = GDALCreateScaledProgress( 0.5, 1.0, pfnProgress,
 *                                                   pProgressData );
 *       GDALDoAnotherOperation( ..., GDALScaledProgress, pScaledProgress );
 *       GDALDestroyScaledProgress( pScaledProgress );
 *
 *       return ...;
 *   }
 * \endcode
 */

void *CPL_STDCALL GDALCreateScaledProgress(double dfMin, double dfMax,
                                           GDALProgressFunc pfnProgress,
                                           void *pData)

{
    if (pfnProgress == nullptr || pfnProgress == GDALDummyProgress)
        return nullptr;

    GDALScaledProgressInfo *psInfo = static_cast<GDALScaledProgressInfo *>(
        CPLCalloc(sizeof(GDALScaledProgressInfo), 1));

    if (std::abs(dfMin - dfMax) < 0.0000001)
        dfMax = dfMin + 0.01;

    psInfo->pData = pData;
    psInfo->pfnProgress = pfnProgress;
    psInfo->dfMin = dfMin;
    psInfo->dfMax = dfMax;

    return static_cast<void *>(psInfo);
}

/************************************************************************/
/*                     GDALDestroyScaledProgress()                      */
/************************************************************************/

/**
 * \brief Cleanup scaled progress handle.
 *
 * This function cleans up the data associated with a scaled progress function
 * as returned by GADLCreateScaledProgress().
 *
 * @param pData scaled progress handle returned by GDALCreateScaledProgress().
 */

void CPL_STDCALL GDALDestroyScaledProgress(void *pData)

{
    CPLFree(pData);
}

/************************************************************************/
/*                          GDALTermProgress()                          */
/************************************************************************/

static constexpr int GDALTermProgressWidth(int nMaxTicks, int nMajorTickSpacing)
{
    int nWidth = 0;
    for (int i = 0; i <= nMaxTicks; i++)
    {
        if (i % nMajorTickSpacing == 0)
        {
            int nPercent = (i * 100) / nMaxTicks;
            do
            {
                nWidth++;
            } while (nPercent /= 10);
        }
        else
        {
            nWidth += 1;
        }
    }
    return nWidth;
}

/**
 * \fn GDALTermProgress(double, const char*, void*)
 * \brief Simple progress report to terminal.
 *
 * This progress reporter prints simple progress report to the
 * terminal window.  The progress report generally looks something like
 * this:

\verbatim
0...10...20...30...40...50...60...70...80...90...100 - done.
\endverbatim

 * Starting with GDAL 3.11, for tasks estimated to take more than 10 seconds,
 * an estimated remaining time is also displayed at the end. And for tasks
 * taking more than 5 seconds to complete, the total time is displayed upon
 * completion.
 *
 * Every 2.5% of progress another number or period is emitted.  Note that
 * GDALTermProgress() uses internal static data to keep track of the last
 * percentage reported and will get confused if two terminal based progress
 * reportings are active at the same time.
 *
 * The GDALTermProgress() function maintains an internal memory of the
 * last percentage complete reported in a static variable, and this makes
 * it unsuitable to have multiple GDALTermProgress()'s active either in a
 * single thread or across multiple threads.
 *
 * @param dfComplete completion ratio from 0.0 to 1.0.
 * @param pszMessage optional message.
 * @param pProgressArg ignored callback data argument.
 *
 * @return Always returns TRUE indicating the process should continue.
 */

int CPL_STDCALL GDALTermProgress(double dfComplete,
                                 CPL_UNUSED const char *pszMessage,
                                 CPL_UNUSED void *pProgressArg)
{
    constexpr int MAX_TICKS = 40;
    constexpr int MAJOR_TICK_SPACING = 4;
    constexpr int LENGTH_OF_0_TO_100_PROGRESS =
        GDALTermProgressWidth(MAX_TICKS, MAJOR_TICK_SPACING);

    const int nThisTick = std::min(
        MAX_TICKS, std::max(0, static_cast<int>(dfComplete * MAX_TICKS)));

    // Have we started a new progress run?
    static int nLastTick = -1;
    static time_t nStartTime = 0;
    static bool bETADisplayed = false;
    if (nThisTick < nLastTick && nLastTick >= MAX_TICKS - 1)
    {
        bETADisplayed = false;
        nLastTick = -1;
    }

    if (nThisTick <= nLastTick)
        return TRUE;

    const time_t nCurTime = time(nullptr);
    if (nLastTick < 0)
        nStartTime = nCurTime;

    constexpr int MIN_DELAY_FOR_ETA = 5;  // in seconds
    if (nCurTime - nStartTime >= MIN_DELAY_FOR_ETA && dfComplete > 0 &&
        dfComplete < 0.5)
    {
        static bool bIsTTY = isatty(static_cast<int>(fileno(stdout))) != 0;
        bETADisplayed = bIsTTY;
    }
    if (bETADisplayed)
    {
        fprintf(stdout, "\r");
        nLastTick = -1;
    }
    int nChars = 0;
    while (nThisTick > nLastTick)
    {
        ++nLastTick;
        if (nLastTick % MAJOR_TICK_SPACING == 0)
        {
            int nPercent = (nLastTick * 100) / MAX_TICKS;
            nChars += nPercent < 10 ? 1 : nPercent >= 100 ? 3 : 2;
            fprintf(stdout, "%d", nPercent);
        }
        else
        {
            ++nChars;
            fprintf(stdout, ".");
        }
    }

    if (nThisTick == MAX_TICKS)
    {
        fprintf(stdout, " - done");
        if (nCurTime - nStartTime >= MIN_DELAY_FOR_ETA)
        {
            const int nEllapsed = static_cast<int>(nCurTime - nStartTime);
            const int nHours = nEllapsed / 3600;
            const int nMins = (nEllapsed % 3600) / 60;
            const int nSecs = nEllapsed % 60;
            fprintf(stdout, " in %02d:%02d:%02d.", nHours, nMins, nSecs);
            if (bETADisplayed)
                fprintf(stdout, "                 ");
        }
        else
            fprintf(stdout, ".");
        fprintf(stdout, "\n");
    }
    else
    {
        if (bETADisplayed)
        {
            const double dfETA =
                (nCurTime - nStartTime) * (1.0 / dfComplete - 1);
            for (int i = nChars; i < LENGTH_OF_0_TO_100_PROGRESS; ++i)
                fprintf(stdout, " ");
            const int nETA = static_cast<int>(dfETA + 0.5);
            const int nHours = nETA / 3600;
            const int nMins = (nETA % 3600) / 60;
            const int nSecs = nETA % 60;
            // 2 extra spaces for extremely long tasks...
            fprintf(stdout, " - estimated remaining time: %02d:%02d:%02d  ",
                    nHours, nMins, nSecs);
        }
        fflush(stdout);
    }

    return TRUE;
}

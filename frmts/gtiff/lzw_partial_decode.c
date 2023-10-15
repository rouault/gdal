/******************************************************************************
 *
 * Project:  GeoTIFF Driver
 * Purpose:  GeoTIFF LZW partial decoder
 * Author:   Even Rouault <even dot rouault at spatialys dot com>
 *
 ******************************************************************************
 * Copyright (c) 2023, Even Rouault <even dot rouault at spatialys dot com>
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

#include "lzw_partial_decode.h"

#include "cpl_conv.h"
#include "cpl_error.h"

#ifdef RENAME_INTERNAL_LIBTIFF_SYMBOLS
#undef RENAME_INTERNAL_LIBTIFF_SYMBOLS
#endif

#define TIFFInitLZW gdal_gtiff_TIFFInitLZW
#define LZWCleanup gdal_gtiff_LZWCleanup
#define LZWDecode gdal_gtiff_LZWDecode
#define LZWDecodeCompat gdal_gtiff_LZWDecodeCompat
#define LZWEncode gdal_gtiff_LZWEncode
#define LZWFixupTags gdal_gtiff_LZWFixupTags
#define LZWPostEncode gdal_gtiff_LZWPostEncode
#define LZWPreDecode gdal_gtiff_LZWPreDecode
#define LZWPreEncode gdal_gtiff_LZWPreEncode
#define LZWSetupDecode gdal_gtiff_LZWSetupDecode
#define LZWSetupEncode gdal_gtiff_LZWSetupEncode
#define TIFFPredictorInit gdal_gtiff_TIFFPredictorInit
#define TIFFPredictorCleanup gdal_gtiff_TIFFPredictorCleanup
#define _TIFFSetDefaultCompressionState                                        \
    gdal_gtiff__TIFFSetDefaultCompressionState
#define TIFFErrorExtR gdal_gtiff_TIFFErrorExtR
#define TIFFWarningExtR gdal_gtiff_TIFFWarningExtR
#define _TIFFmallocExt gdal_gtiff__TIFFmallocExt
#define _TIFFfreeExt gdal_gtiff__TIFFfreeExt
#define TIFFFlushData1 gdal_gtiff_TIFFFlushData1

#include "libtiff/tiffiop.h"

// We do not need predictor to decode first pixel
static int gdal_gtiff_TIFFPredictorInit(TIFF *tiff)
{
    (void)tiff;
    return 0;
}

static int gdal_gtiff_TIFFPredictorCleanup(TIFF *tiff)
{
    (void)tiff;
    return 0;
}

void gdal_gtiff__TIFFSetDefaultCompressionState(TIFF *tiff)
{
    (void)tiff;
}

void gdal_gtiff_TIFFErrorExtR(TIFF *tif, const char *module, const char *fmt,
                              ...)
{
    (void)tif;
    (void)module;

    va_list ap;
    va_start(ap, fmt);
    CPLErrorV(CE_Failure, CPLE_AppDefined, fmt, ap);
    va_end(ap);
}

void gdal_gtiff_TIFFWarningExtR(TIFF *tif, const char *module, const char *fmt,
                                ...)
{
    (void)tif;
    (void)module;

    va_list ap;
    va_start(ap, fmt);
    CPLErrorV(CE_Warning, CPLE_AppDefined, fmt, ap);
    va_end(ap);
}

void *gdal_gtiff__TIFFmallocExt(TIFF *tif, tmsize_t s)
{
    (void)tif;
    return VSIMalloc(s);
}

void gdal_gtiff__TIFFfreeExt(TIFF *tif, void *p)
{
    (void)tif;
    VSIFree(p);
}

int gdal_gtiff_TIFFFlushData1(TIFF *tif)
{
    (void)tif;
    // not reached with decoder
    CPLAssert(0);
    return 0;
}

#define _TIFFmemset memset

#include "libtiff/tif_lzw.c"

LZWDecompressor *GDALGTiffLZWDecompressorAlloc()
{
    TIFF *tif = (TIFF *)CPLCalloc(1, sizeof(TIFF));
    if (TIFFInitLZW(tif, COMPRESSION_LZW) != 1 || LZWSetupDecode(tif) != 1)
    {
        GDALGTiffLZWDecompressorFree((LZWDecompressor *)tif);
        return NULL;
    }
    return (LZWDecompressor *)tif;
}

void GDALGTiffLZWDecompressorFree(LZWDecompressor *dec)
{
    TIFF *tif = (TIFF *)dec;
    if (tif)
    {
        if (tif->tif_data)
            LZWCleanup(tif);
        CPLFree(tif);
    }
}

int GDALGTiffLZWDecompressorDecompressFirstPixel(LZWDecompressor *dec,
                                                 void *inputData,
                                                 size_t inputDataSize,
                                                 void *outputData,
                                                 size_t outputDataSize)
{
    TIFF *tif = (TIFF *)dec;
    tif->tif_rawcc = inputDataSize;
    tif->tif_rawdata = inputData;
    tif->tif_rawcp = inputData;
    LZWPreDecode(tif, 0);
    return LZWDecode(tif, outputData, outputDataSize, 0);
}

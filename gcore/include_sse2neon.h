/******************************************************************************
 *
 * Project:  GDAL
 * Purpose:  Includes sse2neon.h headers
 * Author:   Even Rouault <even dot rouault at spatialys dot com>
 *
 ******************************************************************************
 * Copyright (c) 2024, Even Rouault <even dot rouault at spatialys dot com>
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
 *****************************************************************************/

#ifndef INCLUDE_SSE2NEON_H
#define INCLUDE_SSE2NEON_H

#if defined(__GNUC__)
#pragma GCC system_header
#endif

// This check is done in sse2neon.h just as a warning. Turn that into an
// error, so that gdal.cmake doesn't try to use it
#if !defined(__clang__) && defined(__GNUC__) && __GNUC__ < 10
#error "sse2neon.h: GCC versions earlier than 10 are not supported."
#endif

#include "sse2neon.h"

#ifndef _MM_SHUFFLE2
#define _MM_SHUFFLE2(fp1, fp0) (((fp1) << 1) | (fp0))
#endif

#endif /* INCLUDE_SSE2NEON_H */

/******************************************************************************
 *
 * Project:  GDAL
 * Purpose:  Just-in-time compilation of C code
 * Author:   Even Rouault <even dot rouault at spatialys dot com>
 *
 ******************************************************************************
 * Copyright (c) 2025, Even Rouault <even dot rouault at spatialys dot com>
 *
 * SPDX-License-Identifier: MIT
 ****************************************************************************/

#ifndef GDAL_JIT_H
#define GDAL_JIT_H

#include "cpl_port.h"

#include <functional>
#include <memory>
#include <string>

//! @cond Doxygen_Suppress
// Not directly aimed at being used. Use the GDALGetJITFunction() helper
// instead
typedef void GDALJIT;
std::shared_ptr<GDALJIT> CPL_DLL
GDALCompileAndLoad(const std::string &cCode, const std::string &functionName,
                   uint64_t &functionAddress, std::string *posDisassembledCode);

//! @endcond

/** Returns an executable function from the provided C code.
 *
 * @param cCode Valid C code that has a function called functionName and
 *              whose signature must be FunctionSignature.
 *              The C code must not use any \#include statement.
 * @param functionName Entry point in the C code
 * @param[out] posDisassembledCode Pointer to a string that must receive the
 *                                 disassembly of the compiled code, or nullptr
 *                                 if not useful.
 * @return a std::function of signature FunctionSignature corresponding to the
 *         entry point in the C code (may be invalid in case of error.)
 * @since 3.13
 */
template <typename FunctionSignature>
std::function<FunctionSignature>
    CPL_DLL GDALGetJITFunction(const std::string &cCode,
                               const std::string &functionName,
                               std::string *posDisassembledCode = nullptr)
{
    uint64_t functionAddress = 0;
    std::shared_ptr<GDALJIT> jitHolder = GDALCompileAndLoad(
        cCode, functionName, functionAddress, posDisassembledCode);
    if (!jitHolder || !functionAddress)
        return {};

    using FnPtrType = typename std::add_pointer<FunctionSignature>::type;
    FnPtrType fnPtr = reinterpret_cast<FnPtrType>(functionAddress);

    // We capture the jitHolder by value, because, as it is a shared_ptr, the
    // returned std::function will keep it alive. Which is very important
    // because the raw pointer fnPtr is only valid while *jitHolder is.
    return [fnPtr, jitHolder](auto &&...args)
    { return fnPtr(std::forward<decltype(args)>(args)...); };
}

#endif

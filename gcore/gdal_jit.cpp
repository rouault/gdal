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

#include "gdal_jit.h"

#include "cpl_error.h"
#include "cpl_string.h"

#include <sstream>

//! @cond Doxygen_Suppress

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
#pragma GCC diagnostic ignored "-Wextra-semi"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wredundant-move"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcomma"
#pragma clang diagnostic ignored "-Wdeprecated-copy-with-dtor"
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-unknown-command"
#pragma clang diagnostic ignored "-Winconsistent-missing-destructor-override"
#pragma clang diagnostic ignored "-Wshadow-field"
#pragma clang diagnostic ignored "-Wshorten-64-to-32"
#pragma clang diagnostic ignored "-Wsuggest-destructor-override"
#pragma clang diagnostic ignored "-Wweak-vtables"
#else
// LLVM-17 on /usr/lib/llvm-17/include/clang/AST/TypeLoc.h
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#endif
#endif

#include <clang/Frontend/CompilerInstance.h>
#include <clang/Frontend/CompilerInvocation.h>
#include <clang/CodeGen/CodeGenAction.h>  // EmitLLVMOnlyAction()
#include <clang/Driver/Compilation.h>
#include <clang/Driver/Driver.h>
#include <llvm/ADT/ArrayRef.h>
#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/ExecutionEngine/Orc/ThreadSafeModule.h>  // ThreadSafeContext, ThreadSafeModule
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#if LLVM_VERSION_MAJOR >= 16
#include <llvm/TargetParser/Host.h>
#else
#include <llvm/Support/Host.h>
#endif
#include "llvm/Support/MemoryBuffer.h"
#include <llvm/Support/TargetSelect.h>  // InitializeNativeTarget(), etc.

// Below is for disassembling only
#include "llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h"
#include "llvm/ExecutionEngine/Orc/ObjectTransformLayer.h"
#include "llvm/ExecutionEngine/Orc/Layer.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCInstPrinter.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Object/ObjectFile.h"
#include <llvm/Support/raw_os_ostream.h>
#include <llvm/Support/raw_ostream.h>

/************************************************************************/
/*                       GDALCompileCCodeToIR()                         */
/************************************************************************/

static std::unique_ptr<llvm::Module>
GDALCompileCCodeToIR(llvm::LLVMContext &ctx, bool bHasLibmvec,
                     const std::string &cCode)
{
    auto invocation = std::make_shared<clang::CompilerInvocation>();

#if LLVM_VERSION_MAJOR >= 21
    clang::CompilerInstance compInst(invocation);
    clang::DiagnosticOptions diagOpts;
#else
    clang::CompilerInstance compInst;
    compInst.setInvocation(invocation);
    auto diagOpts = llvm::makeIntrusiveRefCnt<clang::DiagnosticOptions>();
#endif

    auto inMemFS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
    inMemFS->addFile("input.c", 0, llvm::MemoryBuffer::getMemBuffer(cCode));

    auto fileMgr = std::make_unique<clang::FileManager>(
        clang::FileSystemOptions(), inMemFS);
    compInst.setFileManager(fileMgr.release());

    // Create DiagnosticsEngine instance
    auto diags = clang::CompilerInstance::createDiagnostics(
#if LLVM_VERSION_MAJOR >= 20
        *inMemFS,
#endif
#if LLVM_VERSION_MAJOR >= 21
        diagOpts
#else
        diagOpts.get()
#endif
    );

    compInst.setDiagnostics(diags.get());

    const std::string hostCPU(llvm::sys::getHostCPUName());
    const std::string tripleStr(llvm::sys::getProcessTriple());

    clang::driver::Driver driver("clang", tripleStr, compInst.getDiagnostics(),
                                 "clang LLVM compiler", inMemFS.get());

    // clang-format off
    std::vector<const char*> args {
        "clang",
        "-O2",
        "-emit-llvm",
        "-Wall",
        "-Wextra",
        "-Xclang",          "-target-cpu",
        "-Xclang",          hostCPU.c_str(),
        "-x",               "c",
        "-c",               "input.c",
        "-fno-math-errno",
    };
    // clang-format on
    if (bHasLibmvec)
        args.push_back("-fveclib=libmvec");

    auto compilation = std::unique_ptr<clang::driver::Compilation>(
        driver.BuildCompilation(llvm::ArrayRef<const char *>(args)));
    if (!compilation)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Failed to build compilation");
        return nullptr;
    }

    const auto &jobs = compilation->getJobs();

    if (jobs.size() != 1)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "compilation->getJobs() did no return a single job");
        return nullptr;
    }

    const auto &cmd = *jobs.begin();
    clang::CompilerInvocation::CreateFromArgs(*invocation, cmd.getArguments(),
                                              compInst.getDiagnostics());

    clang::EmitLLVMOnlyAction action(&ctx);
    if (!compInst.ExecuteAction(action))
        return nullptr;

    return action.takeModule();
}

/************************************************************************/
/*                     GDALGetObjectDisassembly()                       */
/************************************************************************/

static std::string
GDALGetObjectDisassembly(const llvm::MemoryBuffer &objectBuffer)
{
    llvm::InitializeAllTargetInfos();
    llvm::InitializeAllTargetMCs();
    llvm::InitializeAllDisassemblers();
    llvm::InitializeAllAsmParsers();

    auto objectFilePtr = llvm::object::ObjectFile::createObjectFile(
        objectBuffer.getMemBufferRef());
    if (!objectFilePtr)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "createObjectFile() failed: %s",
                 llvm::toString(objectFilePtr.takeError()).c_str());
        return {};
    }
    const auto &objectFile = **objectFilePtr;
    const auto triple = objectFile.makeTriple();

#if LLVM_VERSION_MAJOR >= 22
    const auto &tripleOrStr = triple;
#else
    const std::string tripleStr = triple.str();
    const auto &tripleOrStr = tripleStr;
#endif

    std::string err;
    const llvm::Target *target =
        llvm::TargetRegistry::lookupTarget(tripleOrStr, err);
    if (!target)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "lookupTarget() failed: %s",
                 err.c_str());
        return {};
    }

    auto regInfo = std::unique_ptr<llvm::MCRegisterInfo>(
        target->createMCRegInfo(tripleOrStr));
    if (!regInfo)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "createMCRegInfo() failed");
        return {};
    }

    llvm::MCTargetOptions targetOptions;
    auto asmInfo = std::unique_ptr<llvm::MCAsmInfo>(
        target->createMCAsmInfo(*regInfo, tripleOrStr, targetOptions));
    if (!asmInfo)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "createMCAsmInfo() failed");
        return {};
    }

    auto instrInfo =
        std::unique_ptr<llvm::MCInstrInfo>(target->createMCInstrInfo());
    if (!instrInfo)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "createMCInstrInfo() failed");
        return {};
    }

    auto subtargetInfo =
        std::unique_ptr<llvm::MCSubtargetInfo>(target->createMCSubtargetInfo(
            tripleOrStr, llvm::StringRef(), llvm::StringRef()));
    if (!subtargetInfo)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "createMCSubtargetInfo() failed");
        return {};
    }

    llvm::MCContext ctx(triple, asmInfo.get(), regInfo.get(),
                        subtargetInfo.get(),
                        /* SourceMgr */ nullptr, &targetOptions,
                        /* DoAutoReset */ false,
                        /* CompilationDir */ llvm::StringRef());

    auto disassembler = std::unique_ptr<llvm::MCDisassembler>(
        target->createMCDisassembler(*subtargetInfo, ctx));
    if (!disassembler)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "createMCDisassembler() failed");
        return {};
    }

    auto instrPrinter = std::unique_ptr<llvm::MCInstPrinter>(
        target->createMCInstPrinter(triple, asmInfo->getAssemblerDialect(),
                                    *asmInfo, *instrInfo, *regInfo));
    if (!instrPrinter)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "createMCInstPrinter() failed");
        return {};
    }

    std::stringstream ss;
    llvm::raw_os_ostream rawOsOstream(ss);

    for (const auto &section : objectFile.sections())
    {
        if (!section.isText())
            continue;

        auto contentsOrErr = section.getContents();
        if (!contentsOrErr)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "section.getContents() failed with %s",
                     llvm::toString(contentsOrErr.takeError()).c_str());
            return {};
        }

        const llvm::StringRef bytesAsStr = *contentsOrErr;
        const uint64_t sectionAddress = section.getAddress();
        const llvm::ArrayRef<uint8_t> bytes(
            reinterpret_cast<const uint8_t *>(bytesAsStr.data()),
            bytesAsStr.size());

        // Iterate over bytes in the section and dump instructions
        size_t idx = 0;
        while (idx < bytes.size())
        {
            llvm::MCInst instruction;
            uint64_t instructionSize = 0;

            const auto instructionAddr = sectionAddress + idx;
            const auto status = disassembler->getInstruction(
                instruction, instructionSize, bytes.slice(idx), instructionAddr,
                llvm::nulls());
            if (status == llvm::MCDisassembler::Success)
            {
                rawOsOstream.write_hex(instructionAddr);
                rawOsOstream << ":\t";

                instrPrinter->printInst(&instruction, instructionAddr, "",
                                        *subtargetInfo, rawOsOstream);

#if defined(__x86_64__) || defined(_M_X64)
                constexpr bool isX86 = true;
#else
                constexpr bool isX86 = false;
#endif
                if constexpr (isX86)
                {
                    // Display absolute address for jumps and calls
                    // Note: that would build for other architectures,
                    // but this is really x86 specific
                    const unsigned opCode = instruction.getOpcode();
                    const llvm::StringRef instName = instrInfo->getName(opCode);

                    if (cpl::starts_with(instName.str(), "J") ||
                        cpl::starts_with(instName.str(), "CALL"))
                    {
                        if (instruction.getNumOperands() > 0)
                        {
                            const auto &operand = instruction.getOperand(0);
                            if (operand.isImm())
                            {
                                const int64_t nJumpOffset = operand.getImm();
                                const uint64_t targetAddr = instructionAddr +
                                                            instructionSize +
                                                            nJumpOffset;
                                rawOsOstream << " <0x"
                                             << llvm::format_hex(targetAddr, 10)
                                             << ">";
                            }
                        }
                    }
                }

                rawOsOstream << "\n";

                idx += instructionSize;
            }
            else
            {
                rawOsOstream << "Could not disassemble one instruction. "
                                "Interrupting disassembly\n";
                break;
            }
        }
    }

    rawOsOstream.flush();
    return ss.str();
}

/************************************************************************/
/*                        GDALCompileAndLoad()                          */
/************************************************************************/

std::shared_ptr<void> GDALCompileAndLoad(const std::string &cCode,
                                         const std::string &functionName,
                                         uint64_t &functionAddress,
                                         std::string *posDisassembledCode)
{
    llvm::InitializeNativeTarget();
    llvm::InitializeNativeTargetAsmPrinter();

    std::vector<std::unique_ptr<llvm::MemoryBuffer>> capturedObjectBuffers;

    const auto objectLinkingLayerCreator =
        [&capturedObjectBuffers](
            llvm::orc::ExecutionSession &executionSession
#if LLVM_VERSION_MAJOR < 21
            ,
            const llvm::Triple &
#endif
            ) -> llvm::Expected<std::unique_ptr<llvm::orc::ObjectLayer>>
    {
        auto baseLinkingLayer =
            std::make_shared<llvm::orc::ObjectLinkingLayer>(executionSession);
        return std::make_unique<llvm::orc::ObjectTransformLayer>(
            executionSession, *(baseLinkingLayer.get()),
            [&capturedObjectBuffers,
             baseLinkingLayer](std::unique_ptr<llvm::MemoryBuffer> memBuffer)
                -> llvm::Expected<std::unique_ptr<llvm::MemoryBuffer>>
            {
                capturedObjectBuffers.push_back(
                    llvm::MemoryBuffer::getMemBufferCopy(memBuffer->getBuffer(),
                                                         "<captured-obj>"));
                return std::move(memBuffer);
            });
    };

    auto jitBuilder = llvm::orc::LLJITBuilder();
    if (posDisassembledCode)
    {
        jitBuilder.setObjectLinkingLayerCreator(objectLinkingLayerCreator);
    }

    auto jitOrErr = jitBuilder.create();
    if (!jitOrErr)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Failed to create LLJIT: %s",
                 llvm::toString(jitOrErr.takeError()).c_str());
        return nullptr;
    }
    auto jit = std::shared_ptr<llvm::orc::LLJIT>(std::move(*jitOrErr));

    bool bHasLibmvec = false;
    const char *name = "libmvec.so.1";
    auto dllGenerator = llvm::orc::DynamicLibrarySearchGenerator::Load(
        name, jit->getDataLayout().getGlobalPrefix());
    if (dllGenerator)
    {
        jit->getMainJITDylib().addGenerator(std::move(*dllGenerator));
        bHasLibmvec = true;
    }
    else
    {
        CPLDebugOnce("VRT", "Cannot load %s: %s", name,
                     llvm::toString(dllGenerator.takeError()).c_str());
    }

    auto llvmContext = std::make_unique<llvm::LLVMContext>();

    auto IRModule =
        GDALCompileCCodeToIR(*(llvmContext.get()), bHasLibmvec, cCode);
    if (!IRModule)
    {
        return nullptr;
    }

    if (jit->addIRModule(llvm::orc::ThreadSafeModule(
            std::move(IRModule),
            llvm::orc::ThreadSafeContext(std::move(llvmContext)))))
    {
        CPLError(CE_Failure, CPLE_AppDefined, "jit->addIRModule() failed");
        return nullptr;
    }

    auto symbol = jit->lookup(functionName);
    if (!symbol)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "jit->lookup(%s) failed",
                 functionName.c_str());
        return nullptr;
    }

#if LLVM_VERSION_MAJOR >= 15
    functionAddress = symbol->getValue();
#else
    functionAddress = symbol->getAddress();
#endif

    if (posDisassembledCode && capturedObjectBuffers.size() == 1)
    {
        *posDisassembledCode =
            GDALGetObjectDisassembly(*capturedObjectBuffers[0]);
    }

    // Type-erase the shared_ptr<llvm::orc::LLJIT>
    return std::static_pointer_cast<GDALJIT>(jit);
}

#if defined(__GNUC__)
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#pragma GCC diagnostic pop
#endif

//! @endcond

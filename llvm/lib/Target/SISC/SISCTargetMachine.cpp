//===-- SISCTargetMachine.cpp - Define TargetMachine for SISC -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Implements the info about SISC target spec.
//
//===----------------------------------------------------------------------===//

#include "SISCTargetMachine.h"
#include "SISC.h"
#include "SISCTargetObjectFile.h"
#include "SISCTargetTransformInfo.h"
#include "TargetInfo/SISCTargetInfo.h"
#include "Utils/SISCBaseInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Target/TargetOptions.h"
using namespace llvm;

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeSISCTarget() {
  RegisterTargetMachine<SISCTargetMachine> X(getTheSISC32Target());
  RegisterTargetMachine<SISCTargetMachine> Y(getTheSISC64Target());
  auto PR = PassRegistry::getPassRegistry();
  initializeGlobalISel(*PR);
  initializeSISCExpandPseudoPass(*PR);
}

static StringRef computeDataLayout(const Triple &TT) {
  if (TT.isArch64Bit()) {
    return "e-m:e-p:64:64-i64:64-i128:128-n64-S128";
  } else {
    assert(TT.isArch32Bit() && "only RV32 and RV64 are currently supported");
    return "e-m:e-p:32:32-i64:64-n32-S128";
  }
}

static Reloc::Model getEffectiveRelocModel(const Triple &TT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue())
    return Reloc::Static;
  return *RM;
}

SISCTargetMachine::SISCTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       Optional<Reloc::Model> RM,
                                       Optional<CodeModel::Model> CM,
                                       CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, computeDataLayout(TT), TT, CPU, FS, Options,
                        getEffectiveRelocModel(TT, RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      TLOF(std::make_unique<SISCELFTargetObjectFile>()) {
  initAsmInfo();

  // RISC-V supports the MachineOutliner.
  setMachineOutliner(true);
}

const SISCSubtarget *
SISCTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  std::string CPU = !CPUAttr.hasAttribute(Attribute::None)
                        ? CPUAttr.getValueAsString().str()
                        : TargetCPU;
  std::string FS = !FSAttr.hasAttribute(Attribute::None)
                       ? FSAttr.getValueAsString().str()
                       : TargetFS;
  std::string Key = CPU + FS;
  auto &I = SubtargetMap[Key];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    auto ABIName = Options.MCOptions.getABIName();
    if (const MDString *ModuleTargetABI = dyn_cast_or_null<MDString>(
            F.getParent()->getModuleFlag("target-abi"))) {
      auto TargetABI = SISCABI::getTargetABI(ABIName);
      if (TargetABI != SISCABI::ABI_Unknown &&
          ModuleTargetABI->getString() != ABIName) {
        report_fatal_error("-target-abi option != target-abi module flag");
      }
      ABIName = ModuleTargetABI->getString();
    }
    I = std::make_unique<SISCSubtarget>(TargetTriple, CPU, FS, ABIName, *this);
  }
  return I.get();
}

TargetTransformInfo
SISCTargetMachine::getTargetTransformInfo(const Function &F) {
  return TargetTransformInfo(SISCTTIImpl(this, F));
}

namespace {
class SISCPassConfig : public TargetPassConfig {
public:
  SISCPassConfig(SISCTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  SISCTargetMachine &getSISCTargetMachine() const {
    return getTM<SISCTargetMachine>();
  }

  void addIRPasses() override;
  bool addInstSelector() override;
  bool addIRTranslator() override;
  bool addLegalizeMachineIR() override;
  bool addRegBankSelect() override;
  bool addGlobalInstructionSelect() override;
  void addPreEmitPass() override;
  void addPreEmitPass2() override;
  void addPreSched2() override;
  void addPreRegAlloc() override;
};
}

TargetPassConfig *SISCTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new SISCPassConfig(*this, PM);
}

void SISCPassConfig::addIRPasses() {
  addPass(createAtomicExpandPass());
  TargetPassConfig::addIRPasses();
}

bool SISCPassConfig::addInstSelector() {
  addPass(createSISCISelDag(getSISCTargetMachine()));

  return false;
}

bool SISCPassConfig::addIRTranslator() {
  addPass(new IRTranslator());
  return false;
}

bool SISCPassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

bool SISCPassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

bool SISCPassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect());
  return false;
}

void SISCPassConfig::addPreSched2() {}

void SISCPassConfig::addPreEmitPass() { addPass(&BranchRelaxationPassID); }

void SISCPassConfig::addPreEmitPass2() {
  addPass(createSISCExpandPseudoPass());
  // Schedule the expansion of AMOs at the last possible moment, avoiding the
  // possibility for other passes to break the requirements for forward
  // progress in the LR/SC block.
  addPass(createSISCExpandAtomicPseudoPass());
}

void SISCPassConfig::addPreRegAlloc() {
  addPass(createSISCMergeBaseOffsetOptPass());
}

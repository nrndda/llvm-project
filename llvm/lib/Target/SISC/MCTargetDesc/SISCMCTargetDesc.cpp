//===-- SISCMCTargetDesc.cpp - SISC Target Descriptions -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// This file provides SISC-specific target descriptions.
///
//===----------------------------------------------------------------------===//

#include "SISCMCTargetDesc.h"
#include "SISCELFStreamer.h"
#include "SISCInstPrinter.h"
#include "SISCMCAsmInfo.h"
#include "SISCTargetStreamer.h"
#include "TargetInfo/SISCTargetInfo.h"
#include "Utils/SISCBaseInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "SISCGenInstrInfo.inc"

#define GET_REGINFO_MC_DESC
#include "SISCGenRegisterInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "SISCGenSubtargetInfo.inc"

using namespace llvm;

static MCInstrInfo *createSISCMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitSISCMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createSISCMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitSISCMCRegisterInfo(X, SISC::X1);
  return X;
}

static MCAsmInfo *createSISCMCAsmInfo(const MCRegisterInfo &MRI,
                                       const Triple &TT,
                                       const MCTargetOptions &Options) {
  MCAsmInfo *MAI = new SISCMCAsmInfo(TT);

  Register SP = MRI.getDwarfRegNum(SISC::X2, true);
  MCCFIInstruction Inst = MCCFIInstruction::cfiDefCfa(nullptr, SP, 0);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCSubtargetInfo *createSISCMCSubtargetInfo(const Triple &TT,
                                                   StringRef CPU, StringRef FS) {
  std::string CPUName = std::string(CPU);
  if (CPUName.empty())
    CPUName = TT.isArch64Bit() ? "generic-rv64" : "generic-rv32";
  return createSISCMCSubtargetInfoImpl(TT, CPUName, FS);
}

static MCInstPrinter *createSISCMCInstPrinter(const Triple &T,
                                               unsigned SyntaxVariant,
                                               const MCAsmInfo &MAI,
                                               const MCInstrInfo &MII,
                                               const MCRegisterInfo &MRI) {
  return new SISCInstPrinter(MAI, MII, MRI);
}

static MCTargetStreamer *
createSISCObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  const Triple &TT = STI.getTargetTriple();
  if (TT.isOSBinFormatELF())
    return new SISCTargetELFStreamer(S, STI);
  return nullptr;
}

static MCTargetStreamer *createSISCAsmTargetStreamer(MCStreamer &S,
                                                      formatted_raw_ostream &OS,
                                                      MCInstPrinter *InstPrint,
                                                      bool isVerboseAsm) {
  return new SISCTargetAsmStreamer(S, OS);
}

static MCTargetStreamer *createSISCNullTargetStreamer(MCStreamer &S) {
  return new SISCTargetStreamer(S);
}

namespace {

class SISCMCInstrAnalysis : public MCInstrAnalysis {
public:
  explicit SISCMCInstrAnalysis(const MCInstrInfo *Info)
      : MCInstrAnalysis(Info) {}

  bool evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size,
                      uint64_t &Target) const override {
    if (isConditionalBranch(Inst)) {
      int64_t Imm;
      if (Size == 2)
        Imm = Inst.getOperand(1).getImm();
      else
        Imm = Inst.getOperand(2).getImm();
      Target = Addr + Imm;
      return true;
    }

    if (Inst.getOpcode() == SISC::C_JAL || Inst.getOpcode() == SISC::C_J) {
      Target = Addr + Inst.getOperand(0).getImm();
      return true;
    }

    if (Inst.getOpcode() == SISC::JAL) {
      Target = Addr + Inst.getOperand(1).getImm();
      return true;
    }

    return false;
  }
};

} // end anonymous namespace

static MCInstrAnalysis *createSISCInstrAnalysis(const MCInstrInfo *Info) {
  return new SISCMCInstrAnalysis(Info);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeSISCTargetMC() {
  for (Target *T : {&getTheSISC32Target(), &getTheSISC64Target()}) {
    TargetRegistry::RegisterMCAsmInfo(*T, createSISCMCAsmInfo);
    TargetRegistry::RegisterMCInstrInfo(*T, createSISCMCInstrInfo);
    TargetRegistry::RegisterMCRegInfo(*T, createSISCMCRegisterInfo);
    TargetRegistry::RegisterMCAsmBackend(*T, createSISCAsmBackend);
    TargetRegistry::RegisterMCCodeEmitter(*T, createSISCMCCodeEmitter);
    TargetRegistry::RegisterMCInstPrinter(*T, createSISCMCInstPrinter);
    TargetRegistry::RegisterMCSubtargetInfo(*T, createSISCMCSubtargetInfo);
    TargetRegistry::RegisterObjectTargetStreamer(
        *T, createSISCObjectTargetStreamer);
    TargetRegistry::RegisterMCInstrAnalysis(*T, createSISCInstrAnalysis);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createSISCAsmTargetStreamer);
    // Register the null target streamer.
    TargetRegistry::RegisterNullTargetStreamer(*T,
                                               createSISCNullTargetStreamer);
  }
}

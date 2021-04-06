//===-- SISCSubtarget.cpp - SISC Subtarget Information ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the SISC specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "SISCSubtarget.h"
#include "SISC.h"
#include "SISCCallLowering.h"
#include "SISCFrameLowering.h"
#include "SISCLegalizerInfo.h"
#include "SISCRegisterBankInfo.h"
#include "SISCTargetMachine.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "sisc-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "SISCGenSubtargetInfo.inc"

void SISCSubtarget::anchor() {}

SISCSubtarget &SISCSubtarget::initializeSubtargetDependencies(
    const Triple &TT, StringRef CPU, StringRef FS, StringRef ABIName) {
  // Determine default and user-specified characteristics
  bool Is64Bit = TT.isArch64Bit();
  std::string CPUName = std::string(CPU);
  if (CPUName.empty())
    CPUName = Is64Bit ? "generic-rv64" : "generic-rv32";
  ParseSubtargetFeatures(CPUName, FS);
  if (Is64Bit) {
    XLenVT = MVT::i64;
    XLen = 64;
  }

  TargetABI = SISCABI::computeTargetABI(TT, getFeatureBits(), ABIName);
  SISCFeatures::validate(TT, getFeatureBits());
  return *this;
}

SISCSubtarget::SISCSubtarget(const Triple &TT, StringRef CPU, StringRef FS,
                               StringRef ABIName, const TargetMachine &TM)
    : SISCGenSubtargetInfo(TT, CPU, FS),
      UserReservedRegister(SISC::NUM_TARGET_REGS),
      FrameLowering(initializeSubtargetDependencies(TT, CPU, FS, ABIName)),
      InstrInfo(*this), RegInfo(getHwMode()), TLInfo(TM, *this) {
  CallLoweringInfo.reset(new SISCCallLowering(*getTargetLowering()));
  Legalizer.reset(new SISCLegalizerInfo(*this));

  auto *RBI = new SISCRegisterBankInfo(*getRegisterInfo());
  RegBankInfo.reset(RBI);
  InstSelector.reset(createSISCInstructionSelector(
      *static_cast<const SISCTargetMachine *>(&TM), *this, *RBI));
}

const CallLowering *SISCSubtarget::getCallLowering() const {
  return CallLoweringInfo.get();
}

InstructionSelector *SISCSubtarget::getInstructionSelector() const {
  return InstSelector.get();
}

const LegalizerInfo *SISCSubtarget::getLegalizerInfo() const {
  return Legalizer.get();
}

const RegisterBankInfo *SISCSubtarget::getRegBankInfo() const {
  return RegBankInfo.get();
}

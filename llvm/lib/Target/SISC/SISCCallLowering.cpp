//===-- SISCCallLowering.cpp - Call lowering -------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// This file implements the lowering of LLVM calls to machine code calls for
/// GlobalISel.
//
//===----------------------------------------------------------------------===//

#include "SISCCallLowering.h"
#include "SISCISelLowering.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"

using namespace llvm;

SISCCallLowering::SISCCallLowering(const SISCTargetLowering &TLI)
    : CallLowering(&TLI) {}

bool SISCCallLowering::lowerReturn(MachineIRBuilder &MIRBuilder,
                                    const Value *Val,
                                    ArrayRef<Register> VRegs) const {

  MachineInstrBuilder Ret = MIRBuilder.buildInstrNoInsert(SISC::PseudoRET);

  if (Val != nullptr) {
    return false;
  }
  MIRBuilder.insertInstr(Ret);
  return true;
}

bool SISCCallLowering::lowerFormalArguments(
    MachineIRBuilder &MIRBuilder, const Function &F,
    ArrayRef<ArrayRef<Register>> VRegs) const {

  if (F.arg_empty())
    return true;

  return false;
}

bool SISCCallLowering::lowerCall(MachineIRBuilder &MIRBuilder,
                                  CallLoweringInfo &Info) const {
  return false;
}

//===-- SISC.h - Top-level interface for SISC -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// RISC-V back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SISC_SISC_H
#define LLVM_LIB_TARGET_SISC_SISC_H

#include "Utils/SISCBaseInfo.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class SISCRegisterBankInfo;
class SISCSubtarget;
class SISCTargetMachine;
class AsmPrinter;
class FunctionPass;
class InstructionSelector;
class MCInst;
class MCOperand;
class MachineInstr;
class MachineOperand;
class PassRegistry;

void LowerSISCMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                    const AsmPrinter &AP);
bool LowerSISCMachineOperandToMCOperand(const MachineOperand &MO,
                                         MCOperand &MCOp, const AsmPrinter &AP);

FunctionPass *createSISCISelDag(SISCTargetMachine &TM);

FunctionPass *createSISCMergeBaseOffsetOptPass();
void initializeSISCMergeBaseOffsetOptPass(PassRegistry &);

FunctionPass *createSISCExpandPseudoPass();
void initializeSISCExpandPseudoPass(PassRegistry &);

FunctionPass *createSISCExpandAtomicPseudoPass();
void initializeSISCExpandAtomicPseudoPass(PassRegistry &);

InstructionSelector *createSISCInstructionSelector(const SISCTargetMachine &,
                                                    SISCSubtarget &,
                                                    SISCRegisterBankInfo &);
}

#endif

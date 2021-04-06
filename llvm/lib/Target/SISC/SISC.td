//===-- SISC.td - Describe the SISC Target Machine -------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// RISC-V subtarget features and instruction predicates.
//===----------------------------------------------------------------------===//

def FeatureRelax
    : SubtargetFeature<"relax", "EnableLinkerRelax", "true",
                       "Enable Linker relaxation.">;

foreach i = {1-31} in
    def FeatureReserveX#i :
        SubtargetFeature<"reserve-x"#i, "UserReservedRegister[SISC::X"#i#"]",
                         "true", "Reserve X"#i>;

def FeatureSaveRestore : SubtargetFeature<"save-restore", "EnableSaveRestore",
                                          "true", "Enable save/restore.">;

//===----------------------------------------------------------------------===//
// Named operands for CSR instructions.
//===----------------------------------------------------------------------===//

include "SISCSystemOperands.td"

//===----------------------------------------------------------------------===//
// Registers, calling conventions, instruction descriptions.
//===----------------------------------------------------------------------===//

include "SISCSchedule.td"
include "SISCRegisterInfo.td"
include "SISCCallingConv.td"
include "SISCInstrInfo.td"
include "SISCRegisterBanks.td"

//===----------------------------------------------------------------------===//
// RISC-V processors supported.
//===----------------------------------------------------------------------===//

def : ProcessorModel<"generic-sisc", NoSchedModel, []>;

//===----------------------------------------------------------------------===//
// Define the RISC-V target.
//===----------------------------------------------------------------------===//

def SISCInstrInfo : InstrInfo {
  let guessInstructionProperties = 0;
}

def SISCAsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let AllowDuplicateRegisterNames = 1;
}

def SISCAsmWriter : AsmWriter {
  int PassSubtarget = 1;
}

def SISC : Target {
  let InstructionSet  =  SISCInstrInfo;
  let AssemblyParsers = [SISCAsmParser];
  let AssemblyWriters = [SISCAsmWriter];
  let AllowRegisterRenaming = 1;
}

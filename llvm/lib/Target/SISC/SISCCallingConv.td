//===-- SISCCallingConv.td - Calling Conventions SISC ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the SISC architecture.
//
//===----------------------------------------------------------------------===//

// The RISC-V calling convention is handled with custom code in
// SISCISelLowering.cpp (CC_SISC).

def CSR_ILP32_LP64
    : CalleeSavedRegs<(add X1, X3, X4, X8, X9, (sequence "X%u", 18, 27))>;

// Needed for implementation of SISCRegisterInfo::getNoPreservedMask()
def CSR_NoRegs : CalleeSavedRegs<(add)>;

// Interrupt handler needs to save/restore all registers that are used,
// both Caller and Callee saved registers.
def CSR_Interrupt : CalleeSavedRegs<(add X1,
    (sequence "X%u", 3, 9),
    (sequence "X%u", 10, 11),
    (sequence "X%u", 12, 17),
    (sequence "X%u", 18, 27),
    (sequence "X%u", 28, 31))>;

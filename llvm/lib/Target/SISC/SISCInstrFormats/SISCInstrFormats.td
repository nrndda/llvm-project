//===-- SISCInst1rFormats.td - SISC Instruction Formats ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//
//  These instruction format definitions are structured to match the
//  description in the RISC-V User-Level ISA specification as closely as
//  possible. For instance, the specification describes instructions with the
//  MSB (31st bit) on the left and the LSB (0th bit) on the right. This is
//  reflected in the order of parameters to each instruction class.
//
//  One area of divergence is in the description of immediates. The
//  specification describes immediate encoding in terms of bit-slicing
//  operations on the logical value represented. The immediate argument to
//  these instruction formats instead represents the bit sequence that will be
//  inserted into the instruction. e.g. although JAL's immediate is logically
//  a 21-bit value (where the LSB is always zero), we describe it as an imm20
//  to match how it is encoded.
//
//===----------------------------------------------------------------------===//

// Format specifies the encoding used by the instruction. This is used by
// SISCMCCodeEmitter to determine which form of fixup to use. These
// definitions must be kept in-sync with SISCBaseInfo.h.
class InstFormat<bits<3> val> {
  bits<3> Value = val;
}
def InstFormatPseudo : InstFormat<0>;
def InstFormatF5     : InstFormat<1>;
def InstFormatF10    : InstFormat<2>;
def InstFormatI5     : InstFormat<3>;
def InstFormatI8     : InstFormat<4>;
def InstFormatI16    : InstFormat<5>;
def InstFormatI32    : InstFormat<6>;

include "SISCInstrFormats/SISCInstrFormatsOp3.td"
include "SISCInstrFormats/SISCInstrFormatsI8.td"

include "SISCInstrFormats/SISCInstrFormatsOp6.td"
include "SISCInstrFormats/SISCInstrFormatsF5.td"
include "SISCInstrFormats/SISCInstrFormatsF10.td"
include "SISCInstrFormats/SISCInstrFormatsI5.td"

// include "SISCInstrFormats/SISCInstrFormats32.td"
// include "SISCInstrFormats/SISCInstrFormatsI16.td"

// include "SISCInstrFormats/SISCInstrFormats48.td"
// include "SISCInstrFormats/SISCInstrFormatsI32.td"

// Pseudo instructions
class Pseudo<dag outs, dag ins, list<dag> pattern, string opcodestr = "", string argstr = "">
    : SISCInst6<outs, ins, opcodestr, argstr, pattern, InstFormatPseudo>, Sched<[]> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;
}

// Pseudo load instructions.
class PseudoLoad<string opcodestr, RegisterClass rdty = GPR>
    : Pseudo<(outs rdty:$rd), (ins bare_symbol:$addr), [], opcodestr, "$rd, $addr"> {
  let hasSideEffects = 0;
  let mayLoad = 1;
  let mayStore = 0;
  let isCodeGenOnly = 0;
  let isAsmParserOnly = 1;
}

// Pseudo store instructions.
class PseudoStore<string opcodestr, RegisterClass rsty = GPR>
    : Pseudo<(outs rsty:$rs, GPR:$tmp), (ins bare_symbol:$addr), [], opcodestr, "$rs, $addr, $tmp"> {
  let hasSideEffects = 0;
  let mayLoad = 0;
  let mayStore = 1;
  let isCodeGenOnly = 0;
  let isAsmParserOnly = 1;
}

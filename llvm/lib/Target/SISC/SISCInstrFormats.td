//===-- SISCInstrFormats.td - SISC Instruction Formats ---*- tablegen -*-===//
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
def InstFormatE      : InstFormat<1>;
def InstFormatS      : InstFormat<2>;
def InstFormatIB     : InstFormat<3>;
def InstFormatR1     : InstFormat<4>;
def InstFormatR2     : InstFormat<5>;

class SISCOpcode0<bits<3> val> {
  bits<3> Value = val;
}
def LIB    : SISCLIBCode<0b111>;

class SISCOpcode1<bits<6> val> {
  bits<6> Value = val;
}

def EPAUSE : SISCOpcode1<0b000000>; def LB : SISCOpcode1<0b000001>; def SOP   : SISCOpcode1<0b000010>; def AND  : SISCOpcode1<0b000001>;
def COPY   : SISCOpcode1<0b001000>; def SB : SISCOpcode1<0b001001>; def SRAI  : SISCOpcode1<0b001010>; def SHA  : SISCOpcode1<0b001001>;
def BEQZ   : SISCOpcode1<0b010000>; def LH : SISCOpcode1<0b010001>; def SLLI  : SISCOpcode1<0b010010>; def SHL  : SISCOpcode1<0b010001>;
def BNEZ   : SISCOpcode1<0b011000>; def SH : SISCOpcode1<0b011001>; def SRLI  : SISCOpcode1<0b011010>; def OR   : SISCOpcode1<0b011001>;
def BLTZ   : SISCOpcode1<0b100000>; def LW : SISCOpcode1<0b100001>; def SLTI  : SISCOpcode1<0b100010>; def SLT  : SISCOpcode1<0b100001>;
def BGEZ   : SISCOpcode1<0b100000>; def SW : SISCOpcode1<0b101001>; def SLTIU : SISCOpcode1<0b101010>; def SLTU : SISCOpcode1<0b101001>;
def JALR   : SISCOpcode1<0b100000>; def LR : SISCOpcode1<0b110001>; def ADDI  : SISCOpcode1<0b110010>; def ADD  : SISCOpcode1<0b110001>;
                                    def SC : SISCOpcode1<0b111001>;                                    def XOR  : SISCOpcode1<0b111001>;

class SISCInst<dag outs, dag ins, string opcodestr, string argstr,
             list<dag> pattern, InstFormat format>
    : Instruction {
  field bits<16> Inst;
  // SoftFail is a field the disassembler can use to provide a way for
  // instructions to not match without killing the whole decode process. It is
  // mainly used for ARM, but Tablegen expects this field to exist or it fails
  // to build the decode table.
  field bits<16> SoftFail = 0;
  let Size = 2;

  bits<6> Opcode = 0;

  let Inst{2-0}   = Opcode{2-0};
  let Inst{15-13} = Opcode{5-3};

  let Namespace = "SISC";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = opcodestr # "\t" # argstr;
  let Pattern = pattern;

  let TSFlags{3-0} = format.Value;
}

// Pseudo instructions
class Pseudo<dag outs, dag ins, list<dag> pattern, string opcodestr = "", string argstr = "">
    : SISCInst<outs, ins, opcodestr, argstr, pattern, InstFormatPseudo>,
      Sched<[]> {
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

class SISCInstE<bits<10> Funct10, SISCOpcode1 opcode, dag outs,
              dag ins, string opcodestr, string argstr>
    : SISCInst<outs, ins, opcodestr, argstr, [], InstFormatE> {
  let Inst{12-3} = Funct10;
  let Opcode = opcode.Value;
}

class SISCInstS<bits<5> Funct5, SISCOpcode1 opcode, dag outs,
              dag ins, string opcodestr, string argstr>
    : SISCInst<outs, ins, opcodestr, argstr, [], InstFormatE> {
  bits<5> rd;

  let Inst{12-8} = Funct5;
  let Inst{7-3} = rd;
  let Opcode = opcode.Value;
}

class SISCInstIB<SISCOpcode0 opcode, dag outs,
              dag ins, string opcodestr, string argstr>
    : SISCInst<outs, ins, opcodestr, argstr, [], InstFormatIB> {
  bits<8> Imm8;
  bits<5> rd;

  let Inst{7-3} = rd;
  let Inst{15-8} = Imm8;
  let Opcode = opcode.Value;
}

class SISCInstR1<SISCOpcode1 opcode, dag outs, dag ins,
               string opcodestr, string argstr>
    : SISCInst<outs, ins, opcodestr, argstr, [], InstFormatR1> {
  bits<5> Imm5;
  bits<5> rd;

  let Inst{12-8} = Imm5;
  let Inst{7-3} = rd;
  let Opcode = opcode.Value;
}

class SISCInstR2<SISCOpcode1 opcode, dag outs, dag ins,
               string opcodestr, string argstr>
    : SISCInst<outs, ins, opcodestr, argstr, [], InstFormatR2> {
  bits<5> rs;
  bits<5> rd;

  let Inst{12-8} = rs;
  let Inst{7-3} = rd;
  let Opcode = opcode.Value;
}

//===-- SISCInst1rFormats.td - SISC Instruction Formats ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

class SISCInstI5<SISCOpcode6 opcode, dag outs, dag ins,
               string opcodestr, string argstr>
    : SISCInst6<outs, ins, opcodestr, argstr, [], InstFormatI5> {
  bits<5> rd;
  bits<5> Imm5;
  
  let Inst{7-3} = rd;
  let Inst{12-8} = Imm5;
  let Opcode = opcode.Value;
}

def OPC_SLLI   : SISCOpcode6<0b001010>;
def OPC_SRLI   : SISCOpcode6<0b010010>;
def OPC_SRAI   : SISCOpcode6<0b011010>;
def OPC_ADDI   : SISCOpcode6<0b110010>;
def OPC_SLTI   : SISCOpcode6<0b100010>;
def OPC_SLTIU  : SISCOpcode6<0b101010>;

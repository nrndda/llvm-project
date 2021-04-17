//===-- SISCInst1rFormats.td - SISC Instruction Formats ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

class SISCInstI16<SISCOpcode6 opcode, dag outs,
              dag ins, string opcodestr, string argstr>
    : SISCInst32<outs, ins, opcodestr, argstr, [], InstFormatI16> {
  bits<16> Imm16;
  bits<5> rd;

  let Inst{7-3} = rd;
  let Inst{31-16} = Imm16;
  let Opcode = opcode.Value;
}

def OPC_LIH : SISCOpcode6<0b000111>;

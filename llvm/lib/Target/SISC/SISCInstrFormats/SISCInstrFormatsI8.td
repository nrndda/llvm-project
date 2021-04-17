//===-- SISCInst1rFormats.td - SISC Instruction Formats ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

class SISCInstI8<SISCOpcode3 opcode, dag outs,
              dag ins, string opcodestr, string argstr>
    : SISCInst3<outs, ins, opcodestr, argstr, [], InstFormatI8> {
  bits<8> Imm8;
  bits<5> rd;

  let Inst{7-3} = rd;
  let Inst{15-8} = Imm8;
  let Opcode = opcode.Value;
}

def OPC_LIB : SISCOpcode3<0b111>;

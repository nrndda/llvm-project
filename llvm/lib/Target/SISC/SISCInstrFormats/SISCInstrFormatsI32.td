//===-- SISCInst1rFormats.td - SISC Instruction Formats ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

class SISCInstI32<SISCOpcode6 opcode, dag outs,
              dag ins, string opcodestr, string argstr>
    : SISCInst6<outs, ins, opcodestr, argstr, [], InstFormatI32> {
  bits<32> Imm32;
  bits<5> rd;

  let Inst{7-3} = rd;
  let Inst{47-16} = Imm32;
  let Opcode = opcode.Value;
}

def OPC_LIW : SISCOpcode6<0b000111>;

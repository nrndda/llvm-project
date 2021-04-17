//===-- SISCInst1rFormats.td - SISC Instruction Formats ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

class SISCInstF5<bits<5> Funct5, SISCOpcode6 opcode, dag outs,
              dag ins, string opcodestr, string argstr>
    : SISCInst6<outs, ins, opcodestr, argstr, [], InstFormatF5> {
  bits<5> rd;

  
  let Inst{7-3} = rd;
  let Inst{12-8} = Funct5;
  let Opcode = opcode.Value;
}

def OPC_SOP    : SISCOpcode6<0b111001>;
def OPC_FCVT   : SISCOpcode6<0b111001>;

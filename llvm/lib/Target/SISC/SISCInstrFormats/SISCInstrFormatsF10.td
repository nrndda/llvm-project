//===-- SISCInst1rFormats.td - SISC Instruction Formats ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

class SISCInstF10<bits<10> Funct10, SISCOpcode6 opcode, dag outs,
              dag ins, string opcodestr, string argstr>
    : SISCInst6<outs, ins, opcodestr, argstr, [], InstFormatF10> {
      
      
      
  let Inst{12-3} = Funct10;
  
  let Opcode = opcode.Value;
}

def OPC_EPAUSE : SISCOpcode6<0b000000>;

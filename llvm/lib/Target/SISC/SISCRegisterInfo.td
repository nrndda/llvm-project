//===-- SISCRegisterInfo.td - RISC-V Register defs --------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the RISC-V register files
//===----------------------------------------------------------------------===//

let Namespace = "SISC" in {
class SISCReg<bits<5> Enc, string n, list<string> alt = []> : Register<n> {
  let HWEncoding{4-0} = Enc;
  let AltNames = alt;
}

class SISCRegWithSubRegs<bits<5> Enc, string n, list<Register> subregs,
                          list<string> alt = []>
      : RegisterWithSubRegs<n, subregs> {
  let HWEncoding{4-0} = Enc;
  let AltNames = alt;
}

def ABIRegAltName : RegAltNameIndex;

} // Namespace = "SISC"

// Integer registers
// CostPerUse is set higher for registers that may not be compressible as they
// are not part of GPRC, the most restrictive register class used by the
// compressed instruction set. This will influence the greedy register
// allocator to reduce the use of registers that can't be encoded in 16 bit
// instructions. This affects register allocation even when compressed
// instruction isn't targeted, we see no major negative codegen impact.

let RegAltNameIndices = [ABIRegAltName] in {
  def X0  : SISCReg<0, "x0", ["zero"]>, DwarfRegNum<[0]>;
  def X1  : SISCReg<1, "x1", ["ra"]>, DwarfRegNum<[1]>;
  def X2  : SISCReg<2, "x2", ["sp"]>, DwarfRegNum<[2]>;
  def X3  : SISCReg<3, "x3", ["gp"]>, DwarfRegNum<[3]>;
  def X4  : SISCReg<4, "x4", ["tp"]>, DwarfRegNum<[4]>;
  def X5  : SISCReg<5, "x5", ["t0"]>, DwarfRegNum<[5]>;
  def X6  : SISCReg<6, "x6", ["t1"]>, DwarfRegNum<[6]>;
  def X7  : SISCReg<7, "x7", ["t2"]>, DwarfRegNum<[7]>;
  def X8  : SISCReg<8, "x8", ["s0", "fp"]>, DwarfRegNum<[8]>;
  def X9  : SISCReg<9, "x9", ["s1"]>, DwarfRegNum<[9]>;
  def X10 : SISCReg<10,"x10", ["a0"]>, DwarfRegNum<[10]>;
  def X11 : SISCReg<11,"x11", ["a1"]>, DwarfRegNum<[11]>;
  def X12 : SISCReg<12,"x12", ["a2"]>, DwarfRegNum<[12]>;
  def X13 : SISCReg<13,"x13", ["a3"]>, DwarfRegNum<[13]>;
  def X14 : SISCReg<14,"x14", ["a4"]>, DwarfRegNum<[14]>;
  def X15 : SISCReg<15,"x15", ["a5"]>, DwarfRegNum<[15]>;
  def X16 : SISCReg<16,"x16", ["a6"]>, DwarfRegNum<[16]>;
  def X17 : SISCReg<17,"x17", ["a7"]>, DwarfRegNum<[17]>;
  def X18 : SISCReg<18,"x18", ["s2"]>, DwarfRegNum<[18]>;
  def X19 : SISCReg<19,"x19", ["s3"]>, DwarfRegNum<[19]>;
  def X20 : SISCReg<20,"x20", ["s4"]>, DwarfRegNum<[20]>;
  def X21 : SISCReg<21,"x21", ["s5"]>, DwarfRegNum<[21]>;
  def X22 : SISCReg<22,"x22", ["s6"]>, DwarfRegNum<[22]>;
  def X23 : SISCReg<23,"x23", ["s7"]>, DwarfRegNum<[23]>;
  def X24 : SISCReg<24,"x24", ["s8"]>, DwarfRegNum<[24]>;
  def X25 : SISCReg<25,"x25", ["s9"]>, DwarfRegNum<[25]>;
  def X26 : SISCReg<26,"x26", ["s10"]>, DwarfRegNum<[26]>;
  def X27 : SISCReg<27,"x27", ["s11"]>, DwarfRegNum<[27]>;
  def X28 : SISCReg<28,"x28", ["t3"]>, DwarfRegNum<[28]>;
  def X29 : SISCReg<29,"x29", ["t4"]>, DwarfRegNum<[29]>;
  def X30 : SISCReg<30,"x30", ["t5"]>, DwarfRegNum<[30]>;
  def X31 : SISCReg<31,"x31", ["t6"]>, DwarfRegNum<[31]>;
}
def XLenVT : ValueTypeByHwMode<[DefaultMode],
                               [i32]>;

// The order of registers represents the preferred allocation sequence.
// Registers are listed in the order caller-save, callee-save, specials.
def GPR : RegisterClass<"SISC", [XLenVT], 32, (add
    (sequence "X%u", 10, 17),
    (sequence "X%u", 5, 7),
    (sequence "X%u", 28, 31),
    (sequence "X%u", 8, 9),
    (sequence "X%u", 18, 27),
    (sequence "X%u", 0, 4)
  )> {
  let RegInfos = RegInfoByHwMode<
      [DefaultMode],
      [RegInfo<32,32,32>]>;
}

// For indirect tail calls, we can't use callee-saved registers, as they are
// restored to the saved value before the tail call, which would clobber a call
// address.
def GPRTC : RegisterClass<"SISC", [XLenVT], 32, (add
    (sequence "X%u", 5, 7),
    (sequence "X%u", 10, 17),
    (sequence "X%u", 28, 31)
  )> {
  let RegInfos = RegInfoByHwMode<
      [DefaultMode],
      [RegInfo<32,32,32>]>;
}

def SP : RegisterClass<"SISC", [XLenVT], 32, (add X2)> {
  let RegInfos = RegInfoByHwMode<
      [DefaultMode],
      [RegInfo<32,32,32>]>;
}

// Floating point registers
let RegAltNameIndices = [ABIRegAltName] in {
  def F0_F  : SISCReg<0, "f0", ["ft0"]>, DwarfRegNum<[32]>;
  def F1_F  : SISCReg<1, "f1", ["ft1"]>, DwarfRegNum<[33]>;
  def F2_F  : SISCReg<2, "f2", ["ft2"]>, DwarfRegNum<[34]>;
  def F3_F  : SISCReg<3, "f3", ["ft3"]>, DwarfRegNum<[35]>;
  def F4_F  : SISCReg<4, "f4", ["ft4"]>, DwarfRegNum<[36]>;
  def F5_F  : SISCReg<5, "f5", ["ft5"]>, DwarfRegNum<[37]>;
  def F6_F  : SISCReg<6, "f6", ["ft6"]>, DwarfRegNum<[38]>;
  def F7_F  : SISCReg<7, "f7", ["ft7"]>, DwarfRegNum<[39]>;
  def F8_F  : SISCReg<8, "f8", ["fs0"]>, DwarfRegNum<[40]>;
  def F9_F  : SISCReg<9, "f9", ["fs1"]>, DwarfRegNum<[41]>;
  def F10_F : SISCReg<10,"f10", ["fa0"]>, DwarfRegNum<[42]>;
  def F11_F : SISCReg<11,"f11", ["fa1"]>, DwarfRegNum<[43]>;
  def F12_F : SISCReg<12,"f12", ["fa2"]>, DwarfRegNum<[44]>;
  def F13_F : SISCReg<13,"f13", ["fa3"]>, DwarfRegNum<[45]>;
  def F14_F : SISCReg<14,"f14", ["fa4"]>, DwarfRegNum<[46]>;
  def F15_F : SISCReg<15,"f15", ["fa5"]>, DwarfRegNum<[47]>;
  def F16_F : SISCReg<16,"f16", ["fa6"]>, DwarfRegNum<[48]>;
  def F17_F : SISCReg<17,"f17", ["fa7"]>, DwarfRegNum<[49]>;
  def F18_F : SISCReg<18,"f18", ["fs2"]>, DwarfRegNum<[50]>;
  def F19_F : SISCReg<19,"f19", ["fs3"]>, DwarfRegNum<[51]>;
  def F20_F : SISCReg<20,"f20", ["fs4"]>, DwarfRegNum<[52]>;
  def F21_F : SISCReg<21,"f21", ["fs5"]>, DwarfRegNum<[53]>;
  def F22_F : SISCReg<22,"f22", ["fs6"]>, DwarfRegNum<[54]>;
  def F23_F : SISCReg<23,"f23", ["fs7"]>, DwarfRegNum<[55]>;
  def F24_F : SISCReg<24,"f24", ["fs8"]>, DwarfRegNum<[56]>;
  def F25_F : SISCReg<25,"f25", ["fs9"]>, DwarfRegNum<[57]>;
  def F26_F : SISCReg<26,"f26", ["fs10"]>, DwarfRegNum<[58]>;
  def F27_F : SISCReg<27,"f27", ["fs11"]>, DwarfRegNum<[59]>;
  def F28_F : SISCReg<28,"f28", ["ft8"]>, DwarfRegNum<[60]>;
  def F29_F : SISCReg<29,"f29", ["ft9"]>, DwarfRegNum<[61]>;
  def F30_F : SISCReg<30,"f30", ["ft10"]>, DwarfRegNum<[62]>;
  def F31_F : SISCReg<31,"f31", ["ft11"]>, DwarfRegNum<[63]>;
}

// The order of registers represents the preferred allocation sequence,
// meaning caller-save regs are listed before callee-save.
def FPR32 : RegisterClass<"SISC", [f32], 32, (add
    (sequence "F%u_F", 0, 7),
    (sequence "F%u_F", 10, 17),
    (sequence "F%u_F", 28, 31),
    (sequence "F%u_F", 8, 9),
    (sequence "F%u_F", 18, 27)
)>;

class RegisterTypes<list<ValueType> reg_types> {
  list<ValueType> types = reg_types;
}

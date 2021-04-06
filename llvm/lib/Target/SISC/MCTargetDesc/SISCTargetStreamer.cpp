//===-- SISCTargetStreamer.cpp - SISC Target Streamer Methods -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides SISC specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "SISCTargetStreamer.h"
#include "SISCSubtarget.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/SISCAttributes.h"

using namespace llvm;

SISCTargetStreamer::SISCTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

void SISCTargetStreamer::finish() { finishAttributeSection(); }

void SISCTargetStreamer::emitDirectiveOptionPush() {}
void SISCTargetStreamer::emitDirectiveOptionPop() {}
void SISCTargetStreamer::emitDirectiveOptionPIC() {}
void SISCTargetStreamer::emitDirectiveOptionNoPIC() {}
void SISCTargetStreamer::emitDirectiveOptionRVC() {}
void SISCTargetStreamer::emitDirectiveOptionNoRVC() {}
void SISCTargetStreamer::emitDirectiveOptionRelax() {}
void SISCTargetStreamer::emitDirectiveOptionNoRelax() {}
void SISCTargetStreamer::emitAttribute(unsigned Attribute, unsigned Value) {}
void SISCTargetStreamer::finishAttributeSection() {}
void SISCTargetStreamer::emitTextAttribute(unsigned Attribute,
                                            StringRef String) {}
void SISCTargetStreamer::emitIntTextAttribute(unsigned Attribute,
                                               unsigned IntValue,
                                               StringRef StringValue) {}

void SISCTargetStreamer::emitTargetAttributes(const MCSubtargetInfo &STI) {
  if (STI.hasFeature(SISC::FeatureRV32E))
    emitAttribute(SISCAttrs::STACK_ALIGN, SISCAttrs::ALIGN_4);
  else
    emitAttribute(SISCAttrs::STACK_ALIGN, SISCAttrs::ALIGN_16);

  std::string Arch = "rv32";
  if (STI.hasFeature(SISC::Feature64Bit))
    Arch = "rv64";
  if (STI.hasFeature(SISC::FeatureRV32E))
    Arch += "e1p9";
  else
    Arch += "i2p0";
  if (STI.hasFeature(SISC::FeatureStdExtM))
    Arch += "_m2p0";
  if (STI.hasFeature(SISC::FeatureStdExtA))
    Arch += "_a2p0";
  if (STI.hasFeature(SISC::FeatureStdExtF))
    Arch += "_f2p0";
  if (STI.hasFeature(SISC::FeatureStdExtD))
    Arch += "_d2p0";
  if (STI.hasFeature(SISC::FeatureStdExtC))
    Arch += "_c2p0";

  emitTextAttribute(SISCAttrs::ARCH, Arch);
}

// This part is for ascii assembly output
SISCTargetAsmStreamer::SISCTargetAsmStreamer(MCStreamer &S,
                                               formatted_raw_ostream &OS)
    : SISCTargetStreamer(S), OS(OS) {}

void SISCTargetAsmStreamer::emitDirectiveOptionPush() {
  OS << "\t.option\tpush\n";
}

void SISCTargetAsmStreamer::emitDirectiveOptionPop() {
  OS << "\t.option\tpop\n";
}

void SISCTargetAsmStreamer::emitDirectiveOptionPIC() {
  OS << "\t.option\tpic\n";
}

void SISCTargetAsmStreamer::emitDirectiveOptionNoPIC() {
  OS << "\t.option\tnopic\n";
}

void SISCTargetAsmStreamer::emitDirectiveOptionRVC() {
  OS << "\t.option\trvc\n";
}

void SISCTargetAsmStreamer::emitDirectiveOptionNoRVC() {
  OS << "\t.option\tnorvc\n";
}

void SISCTargetAsmStreamer::emitDirectiveOptionRelax() {
  OS << "\t.option\trelax\n";
}

void SISCTargetAsmStreamer::emitDirectiveOptionNoRelax() {
  OS << "\t.option\tnorelax\n";
}

void SISCTargetAsmStreamer::emitAttribute(unsigned Attribute, unsigned Value) {
  OS << "\t.attribute\t" << Attribute << ", " << Twine(Value) << "\n";
}

void SISCTargetAsmStreamer::emitTextAttribute(unsigned Attribute,
                                               StringRef String) {
  OS << "\t.attribute\t" << Attribute << ", \"" << String << "\"\n";
}

void SISCTargetAsmStreamer::emitIntTextAttribute(unsigned Attribute,
                                                  unsigned IntValue,
                                                  StringRef StringValue) {}

void SISCTargetAsmStreamer::finishAttributeSection() {}

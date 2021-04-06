//===-- SISCELFStreamer.cpp - SISC ELF Target Streamer Methods ----------===//
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

#include "SISCELFStreamer.h"
#include "MCTargetDesc/SISCAsmBackend.h"
#include "SISCMCTargetDesc.h"
#include "Utils/SISCBaseInfo.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/LEB128.h"
#include "llvm/Support/SISCAttributes.h"

using namespace llvm;

// This part is for ELF object output.
SISCTargetELFStreamer::SISCTargetELFStreamer(MCStreamer &S,
                                               const MCSubtargetInfo &STI)
    : SISCTargetStreamer(S), CurrentVendor("sisc") {
  MCAssembler &MCA = getStreamer().getAssembler();
  const FeatureBitset &Features = STI.getFeatureBits();
  auto &MAB = static_cast<SISCAsmBackend &>(MCA.getBackend());
  SISCABI::ABI ABI = MAB.getTargetABI();
  assert(ABI != SISCABI::ABI_Unknown && "Improperly initialised target ABI");

  unsigned EFlags = MCA.getELFHeaderEFlags();

  if (Features[SISC::FeatureStdExtC])
    EFlags |= ELF::EF_SISC_RVC;

  switch (ABI) {
  case SISCABI::ABI_ILP32:
  case SISCABI::ABI_LP64:
    break;
  case SISCABI::ABI_ILP32F:
  case SISCABI::ABI_LP64F:
    EFlags |= ELF::EF_SISC_FLOAT_ABI_SINGLE;
    break;
  case SISCABI::ABI_ILP32D:
  case SISCABI::ABI_LP64D:
    EFlags |= ELF::EF_SISC_FLOAT_ABI_DOUBLE;
    break;
  case SISCABI::ABI_ILP32E:
    EFlags |= ELF::EF_SISC_RVE;
    break;
  case SISCABI::ABI_Unknown:
    llvm_unreachable("Improperly initialised target ABI");
  }

  MCA.setELFHeaderEFlags(EFlags);
}

MCELFStreamer &SISCTargetELFStreamer::getStreamer() {
  return static_cast<MCELFStreamer &>(Streamer);
}

void SISCTargetELFStreamer::emitDirectiveOptionPush() {}
void SISCTargetELFStreamer::emitDirectiveOptionPop() {}
void SISCTargetELFStreamer::emitDirectiveOptionPIC() {}
void SISCTargetELFStreamer::emitDirectiveOptionNoPIC() {}
void SISCTargetELFStreamer::emitDirectiveOptionRVC() {}
void SISCTargetELFStreamer::emitDirectiveOptionNoRVC() {}
void SISCTargetELFStreamer::emitDirectiveOptionRelax() {}
void SISCTargetELFStreamer::emitDirectiveOptionNoRelax() {}

void SISCTargetELFStreamer::emitAttribute(unsigned Attribute, unsigned Value) {
  setAttributeItem(Attribute, Value, /*OverwriteExisting=*/true);
}

void SISCTargetELFStreamer::emitTextAttribute(unsigned Attribute,
                                               StringRef String) {
  setAttributeItem(Attribute, String, /*OverwriteExisting=*/true);
}

void SISCTargetELFStreamer::emitIntTextAttribute(unsigned Attribute,
                                                  unsigned IntValue,
                                                  StringRef StringValue) {
  setAttributeItems(Attribute, IntValue, StringValue,
                    /*OverwriteExisting=*/true);
}

void SISCTargetELFStreamer::finishAttributeSection() {
  if (Contents.empty())
    return;

  if (AttributeSection) {
    Streamer.SwitchSection(AttributeSection);
  } else {
    MCAssembler &MCA = getStreamer().getAssembler();
    AttributeSection = MCA.getContext().getELFSection(
        ".sisc.attributes", ELF::SHT_SISC_ATTRIBUTES, 0);
    Streamer.SwitchSection(AttributeSection);

    Streamer.emitInt8(ELFAttrs::Format_Version);
  }

  // Vendor size + Vendor name + '\0'
  const size_t VendorHeaderSize = 4 + CurrentVendor.size() + 1;

  // Tag + Tag Size
  const size_t TagHeaderSize = 1 + 4;

  const size_t ContentsSize = calculateContentSize();

  Streamer.emitInt32(VendorHeaderSize + TagHeaderSize + ContentsSize);
  Streamer.emitBytes(CurrentVendor);
  Streamer.emitInt8(0); // '\0'

  Streamer.emitInt8(ELFAttrs::File);
  Streamer.emitInt32(TagHeaderSize + ContentsSize);

  // Size should have been accounted for already, now
  // emit each field as its type (ULEB or String).
  for (AttributeItem item : Contents) {
    Streamer.emitULEB128IntValue(item.Tag);
    switch (item.Type) {
    default:
      llvm_unreachable("Invalid attribute type");
    case AttributeType::Numeric:
      Streamer.emitULEB128IntValue(item.IntValue);
      break;
    case AttributeType::Text:
      Streamer.emitBytes(item.StringValue);
      Streamer.emitInt8(0); // '\0'
      break;
    case AttributeType::NumericAndText:
      Streamer.emitULEB128IntValue(item.IntValue);
      Streamer.emitBytes(item.StringValue);
      Streamer.emitInt8(0); // '\0'
      break;
    }
  }

  Contents.clear();
}

size_t SISCTargetELFStreamer::calculateContentSize() const {
  size_t Result = 0;
  for (AttributeItem item : Contents) {
    switch (item.Type) {
    case AttributeType::Hidden:
      break;
    case AttributeType::Numeric:
      Result += getULEB128Size(item.Tag);
      Result += getULEB128Size(item.IntValue);
      break;
    case AttributeType::Text:
      Result += getULEB128Size(item.Tag);
      Result += item.StringValue.size() + 1; // string + '\0'
      break;
    case AttributeType::NumericAndText:
      Result += getULEB128Size(item.Tag);
      Result += getULEB128Size(item.IntValue);
      Result += item.StringValue.size() + 1; // string + '\0';
      break;
    }
  }
  return Result;
}

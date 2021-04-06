//===-- SISCELFObjectWriter.cpp - SISC ELF Writer -----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/SISCFixupKinds.h"
#include "MCTargetDesc/SISCMCExpr.h"
#include "MCTargetDesc/SISCMCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

namespace {
class SISCELFObjectWriter : public MCELFObjectTargetWriter {
public:
  SISCELFObjectWriter(uint8_t OSABI, bool Is64Bit);

  ~SISCELFObjectWriter() override;

  // Return true if the given relocation must be with a symbol rather than
  // section plus offset.
  bool needsRelocateWithSymbol(const MCSymbol &Sym,
                               unsigned Type) const override {
    // TODO: this is very conservative, update once RISC-V psABI requirements
    //       are clarified.
    return true;
  }

protected:
  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
};
}

SISCELFObjectWriter::SISCELFObjectWriter(uint8_t OSABI, bool Is64Bit)
    : MCELFObjectTargetWriter(Is64Bit, OSABI, ELF::EM_SISC,
                              /*HasRelocationAddend*/ true) {}

SISCELFObjectWriter::~SISCELFObjectWriter() {}

unsigned SISCELFObjectWriter::getRelocType(MCContext &Ctx,
                                            const MCValue &Target,
                                            const MCFixup &Fixup,
                                            bool IsPCRel) const {
  const MCExpr *Expr = Fixup.getValue();
  // Determine the type of the relocation
  unsigned Kind = Fixup.getTargetKind();
  if (Kind >= FirstLiteralRelocationKind)
    return Kind - FirstLiteralRelocationKind;
  if (IsPCRel) {
    switch (Kind) {
    default:
      Ctx.reportError(Fixup.getLoc(), "Unsupported relocation type");
      return ELF::R_SISC_NONE;
    case FK_Data_4:
    case FK_PCRel_4:
      return ELF::R_SISC_32_PCREL;
    case SISC::fixup_sisc_pcrel_hi20:
      return ELF::R_SISC_PCREL_HI20;
    case SISC::fixup_sisc_pcrel_lo12_i:
      return ELF::R_SISC_PCREL_LO12_I;
    case SISC::fixup_sisc_pcrel_lo12_s:
      return ELF::R_SISC_PCREL_LO12_S;
    case SISC::fixup_sisc_got_hi20:
      return ELF::R_SISC_GOT_HI20;
    case SISC::fixup_sisc_tls_got_hi20:
      return ELF::R_SISC_TLS_GOT_HI20;
    case SISC::fixup_sisc_tls_gd_hi20:
      return ELF::R_SISC_TLS_GD_HI20;
    case SISC::fixup_sisc_jal:
      return ELF::R_SISC_JAL;
    case SISC::fixup_sisc_branch:
      return ELF::R_SISC_BRANCH;
    case SISC::fixup_sisc_rvc_jump:
      return ELF::R_SISC_RVC_JUMP;
    case SISC::fixup_sisc_rvc_branch:
      return ELF::R_SISC_RVC_BRANCH;
    case SISC::fixup_sisc_call:
      return ELF::R_SISC_CALL;
    case SISC::fixup_sisc_call_plt:
      return ELF::R_SISC_CALL_PLT;
    }
  }

  switch (Kind) {
  default:
    Ctx.reportError(Fixup.getLoc(), "Unsupported relocation type");
    return ELF::R_SISC_NONE;
  case FK_Data_1:
    Ctx.reportError(Fixup.getLoc(), "1-byte data relocations not supported");
    return ELF::R_SISC_NONE;
  case FK_Data_2:
    Ctx.reportError(Fixup.getLoc(), "2-byte data relocations not supported");
    return ELF::R_SISC_NONE;
  case FK_Data_4:
    if (Expr->getKind() == MCExpr::Target &&
        cast<SISCMCExpr>(Expr)->getKind() == SISCMCExpr::VK_SISC_32_PCREL)
      return ELF::R_SISC_32_PCREL;
    return ELF::R_SISC_32;
  case FK_Data_8:
    return ELF::R_SISC_64;
  case FK_Data_Add_1:
    return ELF::R_SISC_ADD8;
  case FK_Data_Add_2:
    return ELF::R_SISC_ADD16;
  case FK_Data_Add_4:
    return ELF::R_SISC_ADD32;
  case FK_Data_Add_8:
    return ELF::R_SISC_ADD64;
  case FK_Data_Add_6b:
    return ELF::R_SISC_SET6;
  case FK_Data_Sub_1:
    return ELF::R_SISC_SUB8;
  case FK_Data_Sub_2:
    return ELF::R_SISC_SUB16;
  case FK_Data_Sub_4:
    return ELF::R_SISC_SUB32;
  case FK_Data_Sub_8:
    return ELF::R_SISC_SUB64;
  case FK_Data_Sub_6b:
    return ELF::R_SISC_SUB6;
  case SISC::fixup_sisc_hi20:
    return ELF::R_SISC_HI20;
  case SISC::fixup_sisc_lo12_i:
    return ELF::R_SISC_LO12_I;
  case SISC::fixup_sisc_lo12_s:
    return ELF::R_SISC_LO12_S;
  case SISC::fixup_sisc_tprel_hi20:
    return ELF::R_SISC_TPREL_HI20;
  case SISC::fixup_sisc_tprel_lo12_i:
    return ELF::R_SISC_TPREL_LO12_I;
  case SISC::fixup_sisc_tprel_lo12_s:
    return ELF::R_SISC_TPREL_LO12_S;
  case SISC::fixup_sisc_tprel_add:
    return ELF::R_SISC_TPREL_ADD;
  case SISC::fixup_sisc_relax:
    return ELF::R_SISC_RELAX;
  case SISC::fixup_sisc_align:
    return ELF::R_SISC_ALIGN;
  }
}

std::unique_ptr<MCObjectTargetWriter>
llvm::createSISCELFObjectWriter(uint8_t OSABI, bool Is64Bit) {
  return std::make_unique<SISCELFObjectWriter>(OSABI, Is64Bit);
}

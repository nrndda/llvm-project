//===-- SISCMCCodeEmitter.cpp - Convert SISC code to machine code -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the SISCMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/SISCFixupKinds.h"
#include "MCTargetDesc/SISCMCExpr.h"
#include "MCTargetDesc/SISCMCTargetDesc.h"
#include "Utils/SISCBaseInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

STATISTIC(MCNumEmitted, "Number of MC instructions emitted");
STATISTIC(MCNumFixups, "Number of MC fixups created");

namespace {
class SISCMCCodeEmitter : public MCCodeEmitter {
  SISCMCCodeEmitter(const SISCMCCodeEmitter &) = delete;
  void operator=(const SISCMCCodeEmitter &) = delete;
  MCContext &Ctx;
  MCInstrInfo const &MCII;

public:
  SISCMCCodeEmitter(MCContext &ctx, MCInstrInfo const &MCII)
      : Ctx(ctx), MCII(MCII) {}

  ~SISCMCCodeEmitter() override {}

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

  void expandFunctionCall(const MCInst &MI, raw_ostream &OS,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;

  void expandAddTPRel(const MCInst &MI, raw_ostream &OS,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const;

  /// TableGen'erated function for getting the binary encoding for an
  /// instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  /// Return binary encoding of operand. If the machine operand requires
  /// relocation, record the relocation and return zero.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getImmOpValueAsr1(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getImmOpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const;
};
} // end anonymous namespace

MCCodeEmitter *llvm::createSISCMCCodeEmitter(const MCInstrInfo &MCII,
                                              const MCRegisterInfo &MRI,
                                              MCContext &Ctx) {
  return new SISCMCCodeEmitter(Ctx, MCII);
}

// Expand PseudoCALL(Reg), PseudoTAIL and PseudoJump to AUIPC and JALR with
// relocation types. We expand those pseudo-instructions while encoding them,
// meaning AUIPC and JALR won't go through SISC MC to MC compressed
// instruction transformation. This is acceptable because AUIPC has no 16-bit
// form and C_JALR has no immediate operand field.  We let linker relaxation
// deal with it. When linker relaxation is enabled, AUIPC and JALR have a
// chance to relax to JAL.
// If the C extension is enabled, JAL has a chance relax to C_JAL.
void SISCMCCodeEmitter::expandFunctionCall(const MCInst &MI, raw_ostream &OS,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  MCInst TmpInst;
  MCOperand Func;
  Register Ra;
  if (MI.getOpcode() == SISC::PseudoTAIL) {
    Func = MI.getOperand(0);
    Ra = SISC::X6;
  } else if (MI.getOpcode() == SISC::PseudoCALLReg) {
    Func = MI.getOperand(1);
    Ra = MI.getOperand(0).getReg();
  } else if (MI.getOpcode() == SISC::PseudoCALL) {
    Func = MI.getOperand(0);
    Ra = SISC::X1;
  } else if (MI.getOpcode() == SISC::PseudoJump) {
    Func = MI.getOperand(1);
    Ra = MI.getOperand(0).getReg();
  }
  uint32_t Binary;

  assert(Func.isExpr() && "Expected expression");

  const MCExpr *CallExpr = Func.getExpr();

  // Emit AUIPC Ra, Func with R_SISC_CALL relocation type.
  TmpInst = MCInstBuilder(SISC::AUIPC)
                .addReg(Ra)
                .addOperand(MCOperand::createExpr(CallExpr));
  Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  support::endian::write(OS, Binary, support::little);

  if (MI.getOpcode() == SISC::PseudoTAIL ||
      MI.getOpcode() == SISC::PseudoJump)
    // Emit JALR X0, Ra, 0
    TmpInst = MCInstBuilder(SISC::JALR).addReg(SISC::X0).addReg(Ra).addImm(0);
  else
    // Emit JALR Ra, Ra, 0
    TmpInst = MCInstBuilder(SISC::JALR).addReg(Ra).addReg(Ra).addImm(0);
  Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  support::endian::write(OS, Binary, support::little);
}

// Expand PseudoAddTPRel to a simple ADD with the correct relocation.
void SISCMCCodeEmitter::expandAddTPRel(const MCInst &MI, raw_ostream &OS,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  MCOperand DestReg = MI.getOperand(0);
  MCOperand SrcReg = MI.getOperand(1);
  MCOperand TPReg = MI.getOperand(2);
  assert(TPReg.isReg() && TPReg.getReg() == SISC::X4 &&
         "Expected thread pointer as second input to TP-relative add");

  MCOperand SrcSymbol = MI.getOperand(3);
  assert(SrcSymbol.isExpr() &&
         "Expected expression as third input to TP-relative add");

  const SISCMCExpr *Expr = dyn_cast<SISCMCExpr>(SrcSymbol.getExpr());
  assert(Expr && Expr->getKind() == SISCMCExpr::VK_SISC_TPREL_ADD &&
         "Expected tprel_add relocation on TP-relative symbol");

  // Emit the correct tprel_add relocation for the symbol.
  Fixups.push_back(MCFixup::create(
      0, Expr, MCFixupKind(SISC::fixup_sisc_tprel_add), MI.getLoc()));

  // Emit fixup_sisc_relax for tprel_add where the relax feature is enabled.
  if (STI.getFeatureBits()[SISC::FeatureRelax]) {
    const MCConstantExpr *Dummy = MCConstantExpr::create(0, Ctx);
    Fixups.push_back(MCFixup::create(
        0, Dummy, MCFixupKind(SISC::fixup_sisc_relax), MI.getLoc()));
  }

  // Emit a normal ADD instruction with the given operands.
  MCInst TmpInst = MCInstBuilder(SISC::ADD)
                       .addOperand(DestReg)
                       .addOperand(SrcReg)
                       .addOperand(TPReg);
  uint32_t Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  support::endian::write(OS, Binary, support::little);
}

void SISCMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  // Get byte count of instruction.
  unsigned Size = Desc.getSize();

  // SISCInstrInfo::getInstSizeInBytes hard-codes the number of expanded
  // instructions for each pseudo, and must be updated when adding new pseudos
  // or changing existing ones.
  if (MI.getOpcode() == SISC::PseudoCALLReg ||
      MI.getOpcode() == SISC::PseudoCALL ||
      MI.getOpcode() == SISC::PseudoTAIL ||
      MI.getOpcode() == SISC::PseudoJump) {
    expandFunctionCall(MI, OS, Fixups, STI);
    MCNumEmitted += 2;
    return;
  }

  if (MI.getOpcode() == SISC::PseudoAddTPRel) {
    expandAddTPRel(MI, OS, Fixups, STI);
    MCNumEmitted += 1;
    return;
  }

  switch (Size) {
  default:
    llvm_unreachable("Unhandled encodeInstruction length!");
  case 2: {
    uint16_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);
    support::endian::write<uint16_t>(OS, Bits, support::little);
    break;
  }
  case 4: {
    uint32_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);
    support::endian::write(OS, Bits, support::little);
    break;
  }
  }

  ++MCNumEmitted; // Keep track of the # of mi's emitted.
}

unsigned
SISCMCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {

  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());

  if (MO.isImm())
    return static_cast<unsigned>(MO.getImm());

  llvm_unreachable("Unhandled expression!");
  return 0;
}

unsigned
SISCMCCodeEmitter::getImmOpValueAsr1(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isImm()) {
    unsigned Res = MO.getImm();
    assert((Res & 1) == 0 && "LSB is non-zero");
    return Res >> 1;
  }

  return getImmOpValue(MI, OpNo, Fixups, STI);
}

unsigned SISCMCCodeEmitter::getImmOpValue(const MCInst &MI, unsigned OpNo,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  bool EnableRelax = STI.getFeatureBits()[SISC::FeatureRelax];
  const MCOperand &MO = MI.getOperand(OpNo);

  MCInstrDesc const &Desc = MCII.get(MI.getOpcode());
  unsigned MIFrm = Desc.TSFlags & SISCII::InstFormatMask;

  // If the destination is an immediate, there is nothing to do.
  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr() &&
         "getImmOpValue expects only expressions or immediates");
  const MCExpr *Expr = MO.getExpr();
  MCExpr::ExprKind Kind = Expr->getKind();
  SISC::Fixups FixupKind = SISC::fixup_sisc_invalid;
  bool RelaxCandidate = false;
  if (Kind == MCExpr::Target) {
    const SISCMCExpr *RVExpr = cast<SISCMCExpr>(Expr);

    switch (RVExpr->getKind()) {
    case SISCMCExpr::VK_SISC_None:
    case SISCMCExpr::VK_SISC_Invalid:
    case SISCMCExpr::VK_SISC_32_PCREL:
      llvm_unreachable("Unhandled fixup kind!");
    case SISCMCExpr::VK_SISC_TPREL_ADD:
      // tprel_add is only used to indicate that a relocation should be emitted
      // for an add instruction used in TP-relative addressing. It should not be
      // expanded as if representing an actual instruction operand and so to
      // encounter it here is an error.
      llvm_unreachable(
          "VK_SISC_TPREL_ADD should not represent an instruction operand");
    case SISCMCExpr::VK_SISC_LO:
      if (MIFrm == SISCII::InstFormatI)
        FixupKind = SISC::fixup_sisc_lo12_i;
      else if (MIFrm == SISCII::InstFormatS)
        FixupKind = SISC::fixup_sisc_lo12_s;
      else
        llvm_unreachable("VK_SISC_LO used with unexpected instruction format");
      RelaxCandidate = true;
      break;
    case SISCMCExpr::VK_SISC_HI:
      FixupKind = SISC::fixup_sisc_hi20;
      RelaxCandidate = true;
      break;
    case SISCMCExpr::VK_SISC_PCREL_LO:
      if (MIFrm == SISCII::InstFormatI)
        FixupKind = SISC::fixup_sisc_pcrel_lo12_i;
      else if (MIFrm == SISCII::InstFormatS)
        FixupKind = SISC::fixup_sisc_pcrel_lo12_s;
      else
        llvm_unreachable(
            "VK_SISC_PCREL_LO used with unexpected instruction format");
      RelaxCandidate = true;
      break;
    case SISCMCExpr::VK_SISC_PCREL_HI:
      FixupKind = SISC::fixup_sisc_pcrel_hi20;
      RelaxCandidate = true;
      break;
    case SISCMCExpr::VK_SISC_GOT_HI:
      FixupKind = SISC::fixup_sisc_got_hi20;
      break;
    case SISCMCExpr::VK_SISC_TPREL_LO:
      if (MIFrm == SISCII::InstFormatI)
        FixupKind = SISC::fixup_sisc_tprel_lo12_i;
      else if (MIFrm == SISCII::InstFormatS)
        FixupKind = SISC::fixup_sisc_tprel_lo12_s;
      else
        llvm_unreachable(
            "VK_SISC_TPREL_LO used with unexpected instruction format");
      RelaxCandidate = true;
      break;
    case SISCMCExpr::VK_SISC_TPREL_HI:
      FixupKind = SISC::fixup_sisc_tprel_hi20;
      RelaxCandidate = true;
      break;
    case SISCMCExpr::VK_SISC_TLS_GOT_HI:
      FixupKind = SISC::fixup_sisc_tls_got_hi20;
      break;
    case SISCMCExpr::VK_SISC_TLS_GD_HI:
      FixupKind = SISC::fixup_sisc_tls_gd_hi20;
      break;
    case SISCMCExpr::VK_SISC_CALL:
      FixupKind = SISC::fixup_sisc_call;
      RelaxCandidate = true;
      break;
    case SISCMCExpr::VK_SISC_CALL_PLT:
      FixupKind = SISC::fixup_sisc_call_plt;
      RelaxCandidate = true;
      break;
    }
  } else if (Kind == MCExpr::SymbolRef &&
             cast<MCSymbolRefExpr>(Expr)->getKind() == MCSymbolRefExpr::VK_None) {
    if (Desc.getOpcode() == SISC::JAL) {
      FixupKind = SISC::fixup_sisc_jal;
    } else if (MIFrm == SISCII::InstFormatB) {
      FixupKind = SISC::fixup_sisc_branch;
    } else if (MIFrm == SISCII::InstFormatCJ) {
      FixupKind = SISC::fixup_sisc_rvc_jump;
    } else if (MIFrm == SISCII::InstFormatCB) {
      FixupKind = SISC::fixup_sisc_rvc_branch;
    }
  }

  assert(FixupKind != SISC::fixup_sisc_invalid && "Unhandled expression!");

  Fixups.push_back(
      MCFixup::create(0, Expr, MCFixupKind(FixupKind), MI.getLoc()));
  ++MCNumFixups;

  // Ensure an R_SISC_RELAX relocation will be emitted if linker relaxation is
  // enabled and the current fixup will result in a relocation that may be
  // relaxed.
  if (EnableRelax && RelaxCandidate) {
    const MCConstantExpr *Dummy = MCConstantExpr::create(0, Ctx);
    Fixups.push_back(
    MCFixup::create(0, Dummy, MCFixupKind(SISC::fixup_sisc_relax),
                    MI.getLoc()));
    ++MCNumFixups;
  }

  return 0;
}

#include "SISCGenMCCodeEmitter.inc"

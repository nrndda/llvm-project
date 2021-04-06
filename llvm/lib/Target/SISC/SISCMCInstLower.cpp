//===-- SISCMCInstLower.cpp - Convert SISC MachineInstr to an MCInst ------=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower SISC MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "SISC.h"
#include "MCTargetDesc/SISCMCExpr.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

static MCOperand lowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym,
                                    const AsmPrinter &AP) {
  MCContext &Ctx = AP.OutContext;
  SISCMCExpr::VariantKind Kind;

  switch (MO.getTargetFlags()) {
  default:
    llvm_unreachable("Unknown target flag on GV operand");
  case SISCII::MO_None:
    Kind = SISCMCExpr::VK_SISC_None;
    break;
  case SISCII::MO_CALL:
    Kind = SISCMCExpr::VK_SISC_CALL;
    break;
  case SISCII::MO_PLT:
    Kind = SISCMCExpr::VK_SISC_CALL_PLT;
    break;
  case SISCII::MO_LO:
    Kind = SISCMCExpr::VK_SISC_LO;
    break;
  case SISCII::MO_HI:
    Kind = SISCMCExpr::VK_SISC_HI;
    break;
  case SISCII::MO_PCREL_LO:
    Kind = SISCMCExpr::VK_SISC_PCREL_LO;
    break;
  case SISCII::MO_PCREL_HI:
    Kind = SISCMCExpr::VK_SISC_PCREL_HI;
    break;
  case SISCII::MO_GOT_HI:
    Kind = SISCMCExpr::VK_SISC_GOT_HI;
    break;
  case SISCII::MO_TPREL_LO:
    Kind = SISCMCExpr::VK_SISC_TPREL_LO;
    break;
  case SISCII::MO_TPREL_HI:
    Kind = SISCMCExpr::VK_SISC_TPREL_HI;
    break;
  case SISCII::MO_TPREL_ADD:
    Kind = SISCMCExpr::VK_SISC_TPREL_ADD;
    break;
  case SISCII::MO_TLS_GOT_HI:
    Kind = SISCMCExpr::VK_SISC_TLS_GOT_HI;
    break;
  case SISCII::MO_TLS_GD_HI:
    Kind = SISCMCExpr::VK_SISC_TLS_GD_HI;
    break;
  }

  const MCExpr *ME =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, Ctx);

  if (!MO.isJTI() && !MO.isMBB() && MO.getOffset())
    ME = MCBinaryExpr::createAdd(
        ME, MCConstantExpr::create(MO.getOffset(), Ctx), Ctx);

  if (Kind != SISCMCExpr::VK_SISC_None)
    ME = SISCMCExpr::create(ME, Kind, Ctx);
  return MCOperand::createExpr(ME);
}

bool llvm::LowerSISCMachineOperandToMCOperand(const MachineOperand &MO,
                                               MCOperand &MCOp,
                                               const AsmPrinter &AP) {
  switch (MO.getType()) {
  default:
    report_fatal_error("LowerSISCMachineInstrToMCInst: unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit())
      return false;
    MCOp = MCOperand::createReg(MO.getReg());
    break;
  case MachineOperand::MO_RegisterMask:
    // Regmasks are like implicit defs.
    return false;
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = lowerSymbolOperand(MO, MO.getMBB()->getSymbol(), AP);
    break;
  case MachineOperand::MO_GlobalAddress:
    MCOp = lowerSymbolOperand(MO, AP.getSymbol(MO.getGlobal()), AP);
    break;
  case MachineOperand::MO_BlockAddress:
    MCOp = lowerSymbolOperand(
        MO, AP.GetBlockAddressSymbol(MO.getBlockAddress()), AP);
    break;
  case MachineOperand::MO_ExternalSymbol:
    MCOp = lowerSymbolOperand(
        MO, AP.GetExternalSymbolSymbol(MO.getSymbolName()), AP);
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    MCOp = lowerSymbolOperand(MO, AP.GetCPISymbol(MO.getIndex()), AP);
    break;
  }
  return true;
}

void llvm::LowerSISCMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                          const AsmPrinter &AP) {
  OutMI.setOpcode(MI->getOpcode());

  for (const MachineOperand &MO : MI->operands()) {
    MCOperand MCOp;
    if (LowerSISCMachineOperandToMCOperand(MO, MCOp, AP))
      OutMI.addOperand(MCOp);
  }
}

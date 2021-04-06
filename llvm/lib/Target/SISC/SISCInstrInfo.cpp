//===-- SISCInstrInfo.cpp - SISC Instruction Information ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the SISC implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "SISCInstrInfo.h"
#include "SISC.h"
#include "SISCSubtarget.h"
#include "SISCTargetMachine.h"
#include "Utils/SISCMatInt.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GEN_CHECK_COMPRESS_INSTR
#include "SISCGenCompressInstEmitter.inc"

#define GET_INSTRINFO_CTOR_DTOR
#include "SISCGenInstrInfo.inc"

SISCInstrInfo::SISCInstrInfo(SISCSubtarget &STI)
    : SISCGenInstrInfo(SISC::ADJCALLSTACKDOWN, SISC::ADJCALLSTACKUP),
      STI(STI) {}

unsigned SISCInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case SISC::LB:
  case SISC::LBU:
  case SISC::LH:
  case SISC::LHU:
  case SISC::LW:
  case SISC::FLW:
  case SISC::LWU:
  case SISC::LD:
  case SISC::FLD:
    break;
  }

  if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
      MI.getOperand(2).getImm() == 0) {
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  }

  return 0;
}

unsigned SISCInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                            int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case SISC::SB:
  case SISC::SH:
  case SISC::SW:
  case SISC::FSW:
  case SISC::SD:
  case SISC::FSD:
    break;
  }

  if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
      MI.getOperand(2).getImm() == 0) {
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  }

  return 0;
}

void SISCInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 const DebugLoc &DL, MCRegister DstReg,
                                 MCRegister SrcReg, bool KillSrc) const {
  if (SISC::GPRRegClass.contains(DstReg, SrcReg)) {
    BuildMI(MBB, MBBI, DL, get(SISC::ADDI), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addImm(0);
    return;
  }

  // FPR->FPR copies
  unsigned Opc;
  if (SISC::FPR32RegClass.contains(DstReg, SrcReg))
    Opc = SISC::FSGNJ_S;
  else if (SISC::FPR64RegClass.contains(DstReg, SrcReg))
    Opc = SISC::FSGNJ_D;
  else
    llvm_unreachable("Impossible reg-to-reg copy");

  BuildMI(MBB, MBBI, DL, get(Opc), DstReg)
      .addReg(SrcReg, getKillRegState(KillSrc))
      .addReg(SrcReg, getKillRegState(KillSrc));
}

void SISCInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         Register SrcReg, bool IsKill, int FI,
                                         const TargetRegisterClass *RC,
                                         const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  unsigned Opcode;

  if (SISC::GPRRegClass.hasSubClassEq(RC))
    Opcode = TRI->getRegSizeInBits(SISC::GPRRegClass) == 32 ?
             SISC::SW : SISC::SD;
  else if (SISC::FPR32RegClass.hasSubClassEq(RC))
    Opcode = SISC::FSW;
  else if (SISC::FPR64RegClass.hasSubClassEq(RC))
    Opcode = SISC::FSD;
  else
    llvm_unreachable("Can't store this register to stack slot");

  BuildMI(MBB, I, DL, get(Opcode))
      .addReg(SrcReg, getKillRegState(IsKill))
      .addFrameIndex(FI)
      .addImm(0);
}

void SISCInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator I,
                                          Register DstReg, int FI,
                                          const TargetRegisterClass *RC,
                                          const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  unsigned Opcode;

  if (SISC::GPRRegClass.hasSubClassEq(RC))
    Opcode = TRI->getRegSizeInBits(SISC::GPRRegClass) == 32 ?
             SISC::LW : SISC::LD;
  else if (SISC::FPR32RegClass.hasSubClassEq(RC))
    Opcode = SISC::FLW;
  else if (SISC::FPR64RegClass.hasSubClassEq(RC))
    Opcode = SISC::FLD;
  else
    llvm_unreachable("Can't load this register from stack slot");

  BuildMI(MBB, I, DL, get(Opcode), DstReg).addFrameIndex(FI).addImm(0);
}

void SISCInstrInfo::movImm(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MBBI,
                            const DebugLoc &DL, Register DstReg, uint64_t Val,
                            MachineInstr::MIFlag Flag) const {
  MachineFunction *MF = MBB.getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();
  bool IsRV64 = MF->getSubtarget<SISCSubtarget>().is64Bit();
  Register SrcReg = SISC::X0;
  Register Result = MRI.createVirtualRegister(&SISC::GPRRegClass);
  unsigned Num = 0;

  if (!IsRV64 && !isInt<32>(Val))
    report_fatal_error("Should only materialize 32-bit constants for RV32");

  SISCMatInt::InstSeq Seq;
  SISCMatInt::generateInstSeq(Val, IsRV64, Seq);
  assert(Seq.size() > 0);

  for (SISCMatInt::Inst &Inst : Seq) {
    // Write the final result to DstReg if it's the last instruction in the Seq.
    // Otherwise, write the result to the temp register.
    if (++Num == Seq.size())
      Result = DstReg;

    if (Inst.Opc == SISC::LUI) {
      BuildMI(MBB, MBBI, DL, get(SISC::LUI), Result)
          .addImm(Inst.Imm)
          .setMIFlag(Flag);
    } else {
      BuildMI(MBB, MBBI, DL, get(Inst.Opc), Result)
          .addReg(SrcReg, RegState::Kill)
          .addImm(Inst.Imm)
          .setMIFlag(Flag);
    }
    // Only the first instruction has X0 as its source.
    SrcReg = Result;
  }
}

// The contents of values added to Cond are not examined outside of
// SISCInstrInfo, giving us flexibility in what to push to it. For SISC, we
// push BranchOpcode, Reg1, Reg2.
static void parseCondBranch(MachineInstr &LastInst, MachineBasicBlock *&Target,
                            SmallVectorImpl<MachineOperand> &Cond) {
  // Block ends with fall-through condbranch.
  assert(LastInst.getDesc().isConditionalBranch() &&
         "Unknown conditional branch");
  Target = LastInst.getOperand(2).getMBB();
  Cond.push_back(MachineOperand::CreateImm(LastInst.getOpcode()));
  Cond.push_back(LastInst.getOperand(0));
  Cond.push_back(LastInst.getOperand(1));
}

static unsigned getOppositeBranchOpcode(int Opc) {
  switch (Opc) {
  default:
    llvm_unreachable("Unrecognized conditional branch");
  case SISC::BEQ:
    return SISC::BNE;
  case SISC::BNE:
    return SISC::BEQ;
  case SISC::BLT:
    return SISC::BGE;
  case SISC::BGE:
    return SISC::BLT;
  case SISC::BLTU:
    return SISC::BGEU;
  case SISC::BGEU:
    return SISC::BLTU;
  }
}

bool SISCInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                   MachineBasicBlock *&TBB,
                                   MachineBasicBlock *&FBB,
                                   SmallVectorImpl<MachineOperand> &Cond,
                                   bool AllowModify) const {
  TBB = FBB = nullptr;
  Cond.clear();

  // If the block has no terminators, it just falls into the block after it.
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end() || !isUnpredicatedTerminator(*I))
    return false;

  // Count the number of terminators and find the first unconditional or
  // indirect branch.
  MachineBasicBlock::iterator FirstUncondOrIndirectBr = MBB.end();
  int NumTerminators = 0;
  for (auto J = I.getReverse(); J != MBB.rend() && isUnpredicatedTerminator(*J);
       J++) {
    NumTerminators++;
    if (J->getDesc().isUnconditionalBranch() ||
        J->getDesc().isIndirectBranch()) {
      FirstUncondOrIndirectBr = J.getReverse();
    }
  }

  // If AllowModify is true, we can erase any terminators after
  // FirstUncondOrIndirectBR.
  if (AllowModify && FirstUncondOrIndirectBr != MBB.end()) {
    while (std::next(FirstUncondOrIndirectBr) != MBB.end()) {
      std::next(FirstUncondOrIndirectBr)->eraseFromParent();
      NumTerminators--;
    }
    I = FirstUncondOrIndirectBr;
  }

  // We can't handle blocks that end in an indirect branch.
  if (I->getDesc().isIndirectBranch())
    return true;

  // We can't handle blocks with more than 2 terminators.
  if (NumTerminators > 2)
    return true;

  // Handle a single unconditional branch.
  if (NumTerminators == 1 && I->getDesc().isUnconditionalBranch()) {
    TBB = getBranchDestBlock(*I);
    return false;
  }

  // Handle a single conditional branch.
  if (NumTerminators == 1 && I->getDesc().isConditionalBranch()) {
    parseCondBranch(*I, TBB, Cond);
    return false;
  }

  // Handle a conditional branch followed by an unconditional branch.
  if (NumTerminators == 2 && std::prev(I)->getDesc().isConditionalBranch() &&
      I->getDesc().isUnconditionalBranch()) {
    parseCondBranch(*std::prev(I), TBB, Cond);
    FBB = getBranchDestBlock(*I);
    return false;
  }

  // Otherwise, we can't handle this.
  return true;
}

unsigned SISCInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                      int *BytesRemoved) const {
  if (BytesRemoved)
    *BytesRemoved = 0;
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end())
    return 0;

  if (!I->getDesc().isUnconditionalBranch() &&
      !I->getDesc().isConditionalBranch())
    return 0;

  // Remove the branch.
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  I->eraseFromParent();

  I = MBB.end();

  if (I == MBB.begin())
    return 1;
  --I;
  if (!I->getDesc().isConditionalBranch())
    return 1;

  // Remove the branch.
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  I->eraseFromParent();
  return 2;
}

// Inserts a branch into the end of the specific MachineBasicBlock, returning
// the number of instructions inserted.
unsigned SISCInstrInfo::insertBranch(
    MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB,
    ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded) const {
  if (BytesAdded)
    *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 3 || Cond.size() == 0) &&
         "SISC branch conditions have two components!");

  // Unconditional branch.
  if (Cond.empty()) {
    MachineInstr &MI = *BuildMI(&MBB, DL, get(SISC::PseudoBR)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Either a one or two-way conditional branch.
  unsigned Opc = Cond[0].getImm();
  MachineInstr &CondMI =
      *BuildMI(&MBB, DL, get(Opc)).add(Cond[1]).add(Cond[2]).addMBB(TBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(CondMI);

  // One-way conditional branch.
  if (!FBB)
    return 1;

  // Two-way conditional branch.
  MachineInstr &MI = *BuildMI(&MBB, DL, get(SISC::PseudoBR)).addMBB(FBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(MI);
  return 2;
}

unsigned SISCInstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                              MachineBasicBlock &DestBB,
                                              const DebugLoc &DL,
                                              int64_t BrOffset,
                                              RegScavenger *RS) const {
  assert(RS && "RegScavenger required for long branching");
  assert(MBB.empty() &&
         "new block should be inserted for expanding unconditional branch");
  assert(MBB.pred_size() == 1);

  MachineFunction *MF = MBB.getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();

  if (!isInt<32>(BrOffset))
    report_fatal_error(
        "Branch offsets outside of the signed 32-bit range not supported");

  // FIXME: A virtual register must be used initially, as the register
  // scavenger won't work with empty blocks (SIInstrInfo::insertIndirectBranch
  // uses the same workaround).
  Register ScratchReg = MRI.createVirtualRegister(&SISC::GPRRegClass);
  auto II = MBB.end();

  MachineInstr &MI = *BuildMI(MBB, II, DL, get(SISC::PseudoJump))
                          .addReg(ScratchReg, RegState::Define | RegState::Dead)
                          .addMBB(&DestBB, SISCII::MO_CALL);

  RS->enterBasicBlockEnd(MBB);
  unsigned Scav = RS->scavengeRegisterBackwards(SISC::GPRRegClass,
                                                MI.getIterator(), false, 0);
  MRI.replaceRegWith(ScratchReg, Scav);
  MRI.clearVirtRegs();
  RS->setRegUsed(Scav);
  return 8;
}

bool SISCInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert((Cond.size() == 3) && "Invalid branch condition!");
  Cond[0].setImm(getOppositeBranchOpcode(Cond[0].getImm()));
  return false;
}

MachineBasicBlock *
SISCInstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  assert(MI.getDesc().isBranch() && "Unexpected opcode!");
  // The branch target is always the last operand.
  int NumOp = MI.getNumExplicitOperands();
  return MI.getOperand(NumOp - 1).getMBB();
}

bool SISCInstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                           int64_t BrOffset) const {
  unsigned XLen = STI.getXLen();
  // Ideally we could determine the supported branch offset from the
  // SISCII::FormMask, but this can't be used for Pseudo instructions like
  // PseudoBR.
  switch (BranchOp) {
  default:
    llvm_unreachable("Unexpected opcode!");
  case SISC::BEQ:
  case SISC::BNE:
  case SISC::BLT:
  case SISC::BGE:
  case SISC::BLTU:
  case SISC::BGEU:
    return isIntN(13, BrOffset);
  case SISC::JAL:
  case SISC::PseudoBR:
    return isIntN(21, BrOffset);
  case SISC::PseudoJump:
    return isIntN(32, SignExtend64(BrOffset + 0x800, XLen));
  }
}

unsigned SISCInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  unsigned Opcode = MI.getOpcode();

  switch (Opcode) {
  default: {
    if (MI.getParent() && MI.getParent()->getParent()) {
      const auto MF = MI.getMF();
      const auto &TM = static_cast<const SISCTargetMachine &>(MF->getTarget());
      const MCRegisterInfo &MRI = *TM.getMCRegisterInfo();
      const MCSubtargetInfo &STI = *TM.getMCSubtargetInfo();
      const SISCSubtarget &ST = MF->getSubtarget<SISCSubtarget>();
      if (isCompressibleInst(MI, &ST, MRI, STI))
        return 2;
    }
    return get(Opcode).getSize();
  }
  case TargetOpcode::EH_LABEL:
  case TargetOpcode::IMPLICIT_DEF:
  case TargetOpcode::KILL:
  case TargetOpcode::DBG_VALUE:
    return 0;
  // These values are determined based on SISCExpandAtomicPseudoInsts,
  // SISCExpandPseudoInsts and SISCMCCodeEmitter, depending on where the
  // pseudos are expanded.
  case SISC::PseudoCALLReg:
  case SISC::PseudoCALL:
  case SISC::PseudoJump:
  case SISC::PseudoTAIL:
  case SISC::PseudoLLA:
  case SISC::PseudoLA:
  case SISC::PseudoLA_TLS_IE:
  case SISC::PseudoLA_TLS_GD:
    return 8;
  case SISC::PseudoAtomicLoadNand32:
  case SISC::PseudoAtomicLoadNand64:
    return 20;
  case SISC::PseudoMaskedAtomicSwap32:
  case SISC::PseudoMaskedAtomicLoadAdd32:
  case SISC::PseudoMaskedAtomicLoadSub32:
    return 28;
  case SISC::PseudoMaskedAtomicLoadNand32:
    return 32;
  case SISC::PseudoMaskedAtomicLoadMax32:
  case SISC::PseudoMaskedAtomicLoadMin32:
    return 44;
  case SISC::PseudoMaskedAtomicLoadUMax32:
  case SISC::PseudoMaskedAtomicLoadUMin32:
    return 36;
  case SISC::PseudoCmpXchg32:
  case SISC::PseudoCmpXchg64:
    return 16;
  case SISC::PseudoMaskedCmpXchg32:
    return 32;
  case TargetOpcode::INLINEASM:
  case TargetOpcode::INLINEASM_BR: {
    const MachineFunction &MF = *MI.getParent()->getParent();
    const auto &TM = static_cast<const SISCTargetMachine &>(MF.getTarget());
    return getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                              *TM.getMCAsmInfo());
  }
  }
}

bool SISCInstrInfo::isAsCheapAsAMove(const MachineInstr &MI) const {
  const unsigned Opcode = MI.getOpcode();
  switch(Opcode) {
    default:
      break;
    case SISC::ADDI:
    case SISC::ORI:
    case SISC::XORI:
      return (MI.getOperand(1).isReg() && MI.getOperand(1).getReg() == SISC::X0);
  }
  return MI.isAsCheapAsAMove();
}

bool SISCInstrInfo::verifyInstruction(const MachineInstr &MI,
                                       StringRef &ErrInfo) const {
  const MCInstrInfo *MCII = STI.getInstrInfo();
  MCInstrDesc const &Desc = MCII->get(MI.getOpcode());

  for (auto &OI : enumerate(Desc.operands())) {
    unsigned OpType = OI.value().OperandType;
    if (OpType >= SISCOp::OPERAND_FIRST_SISC_IMM &&
        OpType <= SISCOp::OPERAND_LAST_SISC_IMM) {
      const MachineOperand &MO = MI.getOperand(OI.index());
      if (MO.isImm()) {
        int64_t Imm = MO.getImm();
        bool Ok;
        switch (OpType) {
        default:
          llvm_unreachable("Unexpected operand type");
        case SISCOp::OPERAND_UIMM4:
          Ok = isUInt<4>(Imm);
          break;
        case SISCOp::OPERAND_UIMM5:
          Ok = isUInt<5>(Imm);
          break;
        case SISCOp::OPERAND_UIMM12:
          Ok = isUInt<12>(Imm);
          break;
        case SISCOp::OPERAND_SIMM12:
          Ok = isInt<12>(Imm);
          break;
        case SISCOp::OPERAND_SIMM13_LSB0:
          Ok = isShiftedInt<12, 1>(Imm);
          break;
        case SISCOp::OPERAND_UIMM20:
          Ok = isUInt<20>(Imm);
          break;
        case SISCOp::OPERAND_SIMM21_LSB0:
          Ok = isShiftedInt<20, 1>(Imm);
          break;
        case SISCOp::OPERAND_UIMMLOG2XLEN:
          if (STI.getTargetTriple().isArch64Bit())
            Ok = isUInt<6>(Imm);
          else
            Ok = isUInt<5>(Imm);
          break;
        }
        if (!Ok) {
          ErrInfo = "Invalid immediate";
          return false;
        }
      }
    }
  }

  return true;
}

// Return true if get the base operand, byte offset of an instruction and the
// memory width. Width is the size of memory that is being loaded/stored.
bool SISCInstrInfo::getMemOperandWithOffsetWidth(
    const MachineInstr &LdSt, const MachineOperand *&BaseReg, int64_t &Offset,
    unsigned &Width, const TargetRegisterInfo *TRI) const {
  if (!LdSt.mayLoadOrStore())
    return false;

  // Here we assume the standard RISC-V ISA, which uses a base+offset
  // addressing mode. You'll need to relax these conditions to support custom
  // load/stores instructions.
  if (LdSt.getNumExplicitOperands() != 3)
    return false;
  if (!LdSt.getOperand(1).isReg() || !LdSt.getOperand(2).isImm())
    return false;

  if (!LdSt.hasOneMemOperand())
    return false;

  Width = (*LdSt.memoperands_begin())->getSize();
  BaseReg = &LdSt.getOperand(1);
  Offset = LdSt.getOperand(2).getImm();
  return true;
}

bool SISCInstrInfo::areMemAccessesTriviallyDisjoint(
    const MachineInstr &MIa, const MachineInstr &MIb) const {
  assert(MIa.mayLoadOrStore() && "MIa must be a load or store.");
  assert(MIb.mayLoadOrStore() && "MIb must be a load or store.");

  if (MIa.hasUnmodeledSideEffects() || MIb.hasUnmodeledSideEffects() ||
      MIa.hasOrderedMemoryRef() || MIb.hasOrderedMemoryRef())
    return false;

  // Retrieve the base register, offset from the base register and width. Width
  // is the size of memory that is being loaded/stored (e.g. 1, 2, 4).  If
  // base registers are identical, and the offset of a lower memory access +
  // the width doesn't overlap the offset of a higher memory access,
  // then the memory accesses are different.
  const TargetRegisterInfo *TRI = STI.getRegisterInfo();
  const MachineOperand *BaseOpA = nullptr, *BaseOpB = nullptr;
  int64_t OffsetA = 0, OffsetB = 0;
  unsigned int WidthA = 0, WidthB = 0;
  if (getMemOperandWithOffsetWidth(MIa, BaseOpA, OffsetA, WidthA, TRI) &&
      getMemOperandWithOffsetWidth(MIb, BaseOpB, OffsetB, WidthB, TRI)) {
    if (BaseOpA->isIdenticalTo(*BaseOpB)) {
      int LowOffset = std::min(OffsetA, OffsetB);
      int HighOffset = std::max(OffsetA, OffsetB);
      int LowWidth = (LowOffset == OffsetA) ? WidthA : WidthB;
      if (LowOffset + LowWidth <= HighOffset)
        return true;
    }
  }
  return false;
}

std::pair<unsigned, unsigned>
SISCInstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  const unsigned Mask = SISCII::MO_DIRECT_FLAG_MASK;
  return std::make_pair(TF & Mask, TF & ~Mask);
}

ArrayRef<std::pair<unsigned, const char *>>
SISCInstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  using namespace SISCII;
  static const std::pair<unsigned, const char *> TargetFlags[] = {
      {MO_CALL, "sisc-call"},
      {MO_PLT, "sisc-plt"},
      {MO_LO, "sisc-lo"},
      {MO_HI, "sisc-hi"},
      {MO_PCREL_LO, "sisc-pcrel-lo"},
      {MO_PCREL_HI, "sisc-pcrel-hi"},
      {MO_GOT_HI, "sisc-got-hi"},
      {MO_TPREL_LO, "sisc-tprel-lo"},
      {MO_TPREL_HI, "sisc-tprel-hi"},
      {MO_TPREL_ADD, "sisc-tprel-add"},
      {MO_TLS_GOT_HI, "sisc-tls-got-hi"},
      {MO_TLS_GD_HI, "sisc-tls-gd-hi"}};
  return makeArrayRef(TargetFlags);
}
bool SISCInstrInfo::isFunctionSafeToOutlineFrom(
    MachineFunction &MF, bool OutlineFromLinkOnceODRs) const {
  const Function &F = MF.getFunction();

  // Can F be deduplicated by the linker? If it can, don't outline from it.
  if (!OutlineFromLinkOnceODRs && F.hasLinkOnceODRLinkage())
    return false;

  // Don't outline from functions with section markings; the program could
  // expect that all the code is in the named section.
  if (F.hasSection())
    return false;

  // It's safe to outline from MF.
  return true;
}

bool SISCInstrInfo::isMBBSafeToOutlineFrom(MachineBasicBlock &MBB,
                                            unsigned &Flags) const {
  // More accurate safety checking is done in getOutliningCandidateInfo.
  return true;
}

// Enum values indicating how an outlined call should be constructed.
enum MachineOutlinerConstructionID {
  MachineOutlinerDefault
};

outliner::OutlinedFunction SISCInstrInfo::getOutliningCandidateInfo(
    std::vector<outliner::Candidate> &RepeatedSequenceLocs) const {

  // First we need to filter out candidates where the X5 register (IE t0) can't
  // be used to setup the function call.
  auto CannotInsertCall = [](outliner::Candidate &C) {
    const TargetRegisterInfo *TRI = C.getMF()->getSubtarget().getRegisterInfo();

    C.initLRU(*TRI);
    LiveRegUnits LRU = C.LRU;
    return !LRU.available(SISC::X5);
  };

  RepeatedSequenceLocs.erase(std::remove_if(RepeatedSequenceLocs.begin(),
                                            RepeatedSequenceLocs.end(),
                                            CannotInsertCall),
                             RepeatedSequenceLocs.end());

  // If the sequence doesn't have enough candidates left, then we're done.
  if (RepeatedSequenceLocs.size() < 2)
    return outliner::OutlinedFunction();

  unsigned SequenceSize = 0;

  auto I = RepeatedSequenceLocs[0].front();
  auto E = std::next(RepeatedSequenceLocs[0].back());
  for (; I != E; ++I)
    SequenceSize += getInstSizeInBytes(*I);

  // call t0, function = 8 bytes.
  unsigned CallOverhead = 8;
  for (auto &C : RepeatedSequenceLocs)
    C.setCallInfo(MachineOutlinerDefault, CallOverhead);

  // jr t0 = 4 bytes, 2 bytes if compressed instructions are enabled.
  unsigned FrameOverhead = 4;
  if (RepeatedSequenceLocs[0].getMF()->getSubtarget()
          .getFeatureBits()[SISC::FeatureStdExtC])
    FrameOverhead = 2;

  return outliner::OutlinedFunction(RepeatedSequenceLocs, SequenceSize,
                                    FrameOverhead, MachineOutlinerDefault);
}

outliner::InstrType
SISCInstrInfo::getOutliningType(MachineBasicBlock::iterator &MBBI,
                                 unsigned Flags) const {
  MachineInstr &MI = *MBBI;
  MachineBasicBlock *MBB = MI.getParent();
  const TargetRegisterInfo *TRI =
      MBB->getParent()->getSubtarget().getRegisterInfo();

  // Positions generally can't safely be outlined.
  if (MI.isPosition()) {
    // We can manually strip out CFI instructions later.
    if (MI.isCFIInstruction())
      return outliner::InstrType::Invisible;

    return outliner::InstrType::Illegal;
  }

  // Don't trust the user to write safe inline assembly.
  if (MI.isInlineAsm())
    return outliner::InstrType::Illegal;

  // We can't outline branches to other basic blocks.
  if (MI.isTerminator() && !MBB->succ_empty())
    return outliner::InstrType::Illegal;

  // We need support for tail calls to outlined functions before return
  // statements can be allowed.
  if (MI.isReturn())
    return outliner::InstrType::Illegal;

  // Don't allow modifying the X5 register which we use for return addresses for
  // these outlined functions.
  if (MI.modifiesRegister(SISC::X5, TRI) ||
      MI.getDesc().hasImplicitDefOfPhysReg(SISC::X5))
    return outliner::InstrType::Illegal;

  // Make sure the operands don't reference something unsafe.
  for (const auto &MO : MI.operands())
    if (MO.isMBB() || MO.isBlockAddress() || MO.isCPI())
      return outliner::InstrType::Illegal;

  // Don't allow instructions which won't be materialized to impact outlining
  // analysis.
  if (MI.isMetaInstruction())
    return outliner::InstrType::Invisible;

  return outliner::InstrType::Legal;
}

void SISCInstrInfo::buildOutlinedFrame(
    MachineBasicBlock &MBB, MachineFunction &MF,
    const outliner::OutlinedFunction &OF) const {

  // Strip out any CFI instructions
  bool Changed = true;
  while (Changed) {
    Changed = false;
    auto I = MBB.begin();
    auto E = MBB.end();
    for (; I != E; ++I) {
      if (I->isCFIInstruction()) {
        I->removeFromParent();
        Changed = true;
        break;
      }
    }
  }

  MBB.addLiveIn(SISC::X5);

  // Add in a return instruction to the end of the outlined frame.
  MBB.insert(MBB.end(), BuildMI(MF, DebugLoc(), get(SISC::JALR))
      .addReg(SISC::X0, RegState::Define)
      .addReg(SISC::X5)
      .addImm(0));
}

MachineBasicBlock::iterator SISCInstrInfo::insertOutlinedCall(
    Module &M, MachineBasicBlock &MBB, MachineBasicBlock::iterator &It,
    MachineFunction &MF, const outliner::Candidate &C) const {

  // Add in a call instruction to the outlined function at the given location.
  It = MBB.insert(It,
                  BuildMI(MF, DebugLoc(), get(SISC::PseudoCALLReg), SISC::X5)
                      .addGlobalAddress(M.getNamedValue(MF.getName()), 0,
                                        SISCII::MO_CALL));
  return It;
}

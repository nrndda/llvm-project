//===-- SISCFrameLowering.cpp - SISC Frame Information ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the SISC implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "SISCFrameLowering.h"
#include "SISCMachineFunctionInfo.h"
#include "SISCSubtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/IR/DiagnosticInfo.h"
#include "llvm/MC/MCDwarf.h"

using namespace llvm;

// Get the ID of the libcall used for spilling and restoring callee saved
// registers. The ID is representative of the number of registers saved or
// restored by the libcall, except it is zero-indexed - ID 0 corresponds to a
// single register.
static int getLibCallID(const MachineFunction &MF,
                        const std::vector<CalleeSavedInfo> &CSI) {
  const auto *RVFI = MF.getInfo<SISCMachineFunctionInfo>();

  if (CSI.empty() || !RVFI->useSaveRestoreLibCalls(MF))
    return -1;

  Register MaxReg = SISC::NoRegister;
  for (auto &CS : CSI)
    // SISCRegisterInfo::hasReservedSpillSlot assigns negative frame indexes to
    // registers which can be saved by libcall.
    if (CS.getFrameIdx() < 0)
      MaxReg = std::max(MaxReg.id(), CS.getReg());

  if (MaxReg == SISC::NoRegister)
    return -1;

  switch (MaxReg) {
  default:
    llvm_unreachable("Something has gone wrong!");
  case /*s11*/ SISC::X27: return 12;
  case /*s10*/ SISC::X26: return 11;
  case /*s9*/  SISC::X25: return 10;
  case /*s8*/  SISC::X24: return 9;
  case /*s7*/  SISC::X23: return 8;
  case /*s6*/  SISC::X22: return 7;
  case /*s5*/  SISC::X21: return 6;
  case /*s4*/  SISC::X20: return 5;
  case /*s3*/  SISC::X19: return 4;
  case /*s2*/  SISC::X18: return 3;
  case /*s1*/  SISC::X9:  return 2;
  case /*s0*/  SISC::X8:  return 1;
  case /*ra*/  SISC::X1:  return 0;
  }
}

// Get the name of the libcall used for spilling callee saved registers.
// If this function will not use save/restore libcalls, then return a nullptr.
static const char *
getSpillLibCallName(const MachineFunction &MF,
                    const std::vector<CalleeSavedInfo> &CSI) {
  static const char *const SpillLibCalls[] = {
    "__sisc_save_0",
    "__sisc_save_1",
    "__sisc_save_2",
    "__sisc_save_3",
    "__sisc_save_4",
    "__sisc_save_5",
    "__sisc_save_6",
    "__sisc_save_7",
    "__sisc_save_8",
    "__sisc_save_9",
    "__sisc_save_10",
    "__sisc_save_11",
    "__sisc_save_12"
  };

  int LibCallID = getLibCallID(MF, CSI);
  if (LibCallID == -1)
    return nullptr;
  return SpillLibCalls[LibCallID];
}

// Get the name of the libcall used for restoring callee saved registers.
// If this function will not use save/restore libcalls, then return a nullptr.
static const char *
getRestoreLibCallName(const MachineFunction &MF,
                      const std::vector<CalleeSavedInfo> &CSI) {
  static const char *const RestoreLibCalls[] = {
    "__sisc_restore_0",
    "__sisc_restore_1",
    "__sisc_restore_2",
    "__sisc_restore_3",
    "__sisc_restore_4",
    "__sisc_restore_5",
    "__sisc_restore_6",
    "__sisc_restore_7",
    "__sisc_restore_8",
    "__sisc_restore_9",
    "__sisc_restore_10",
    "__sisc_restore_11",
    "__sisc_restore_12"
  };

  int LibCallID = getLibCallID(MF, CSI);
  if (LibCallID == -1)
    return nullptr;
  return RestoreLibCalls[LibCallID];
}

bool SISCFrameLowering::hasFP(const MachineFunction &MF) const {
  const TargetRegisterInfo *RegInfo = MF.getSubtarget().getRegisterInfo();

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
         RegInfo->needsStackRealignment(MF) || MFI.hasVarSizedObjects() ||
         MFI.isFrameAddressTaken();
}

bool SISCFrameLowering::hasBP(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo *TRI = STI.getRegisterInfo();

  return MFI.hasVarSizedObjects() && TRI->needsStackRealignment(MF);
}

// Determines the size of the frame and maximum call frame size.
void SISCFrameLowering::determineFrameLayout(MachineFunction &MF) const {
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const SISCRegisterInfo *RI = STI.getRegisterInfo();

  // Get the number of bytes to allocate from the FrameInfo.
  uint64_t FrameSize = MFI.getStackSize();

  // Get the alignment.
  Align StackAlign = getStackAlign();
  if (RI->needsStackRealignment(MF)) {
    Align MaxStackAlign = std::max(StackAlign, MFI.getMaxAlign());
    FrameSize += (MaxStackAlign.value() - StackAlign.value());
    StackAlign = MaxStackAlign;
  }

  // Set Max Call Frame Size
  uint64_t MaxCallSize = alignTo(MFI.getMaxCallFrameSize(), StackAlign);
  MFI.setMaxCallFrameSize(MaxCallSize);

  // Make sure the frame is aligned.
  FrameSize = alignTo(FrameSize, StackAlign);

  // Update frame info.
  MFI.setStackSize(FrameSize);
}

void SISCFrameLowering::adjustReg(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator MBBI,
                                   const DebugLoc &DL, Register DestReg,
                                   Register SrcReg, int64_t Val,
                                   MachineInstr::MIFlag Flag) const {
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  const SISCInstrInfo *TII = STI.getInstrInfo();

  if (DestReg == SrcReg && Val == 0)
    return;

  if (isInt<12>(Val)) {
    BuildMI(MBB, MBBI, DL, TII->get(SISC::ADDI), DestReg)
        .addReg(SrcReg)
        .addImm(Val)
        .setMIFlag(Flag);
  } else {
    unsigned Opc = SISC::ADD;
    bool isSub = Val < 0;
    if (isSub) {
      Val = -Val;
      Opc = SISC::SUB;
    }

    Register ScratchReg = MRI.createVirtualRegister(&SISC::GPRRegClass);
    TII->movImm(MBB, MBBI, DL, ScratchReg, Val, Flag);
    BuildMI(MBB, MBBI, DL, TII->get(Opc), DestReg)
        .addReg(SrcReg)
        .addReg(ScratchReg, RegState::Kill)
        .setMIFlag(Flag);
  }
}

// Returns the register used to hold the frame pointer.
static Register getFPReg(const SISCSubtarget &STI) { return SISC::X8; }

// Returns the register used to hold the stack pointer.
static Register getSPReg(const SISCSubtarget &STI) { return SISC::X2; }

static SmallVector<CalleeSavedInfo, 8>
getNonLibcallCSI(const std::vector<CalleeSavedInfo> &CSI) {
  SmallVector<CalleeSavedInfo, 8> NonLibcallCSI;

  for (auto &CS : CSI)
    if (CS.getFrameIdx() >= 0)
      NonLibcallCSI.push_back(CS);

  return NonLibcallCSI;
}

void SISCFrameLowering::emitPrologue(MachineFunction &MF,
                                      MachineBasicBlock &MBB) const {
  MachineFrameInfo &MFI = MF.getFrameInfo();
  auto *RVFI = MF.getInfo<SISCMachineFunctionInfo>();
  const SISCRegisterInfo *RI = STI.getRegisterInfo();
  const SISCInstrInfo *TII = STI.getInstrInfo();
  MachineBasicBlock::iterator MBBI = MBB.begin();

  Register FPReg = getFPReg(STI);
  Register SPReg = getSPReg(STI);
  Register BPReg = SISCABI::getBPReg();

  // Since spillCalleeSavedRegisters may have inserted a libcall, skip past
  // any instructions marked as FrameSetup
  while (MBBI != MBB.end() && MBBI->getFlag(MachineInstr::FrameSetup))
    ++MBBI;

  // Debug location must be unknown since the first debug location is used
  // to determine the end of the prologue.
  DebugLoc DL;

  // Determine the correct frame layout
  determineFrameLayout(MF);

  // If libcalls are used to spill and restore callee-saved registers, the frame
  // has two sections; the opaque section managed by the libcalls, and the
  // section managed by MachineFrameInfo which can also hold callee saved
  // registers in fixed stack slots, both of which have negative frame indices.
  // This gets even more complicated when incoming arguments are passed via the
  // stack, as these too have negative frame indices. An example is detailed
  // below:
  //
  //  | incoming arg | <- FI[-3]
  //  | libcallspill |
  //  | calleespill  | <- FI[-2]
  //  | calleespill  | <- FI[-1]
  //  | this_frame   | <- FI[0]
  //
  // For negative frame indices, the offset from the frame pointer will differ
  // depending on which of these groups the frame index applies to.
  // The following calculates the correct offset knowing the number of callee
  // saved registers spilt by the two methods.
  if (int LibCallRegs = getLibCallID(MF, MFI.getCalleeSavedInfo()) + 1) {
    // Calculate the size of the frame managed by the libcall. The libcalls are
    // implemented such that the stack will always be 16 byte aligned.
    unsigned LibCallFrameSize = alignTo((STI.getXLen() / 8) * LibCallRegs, 16);
    RVFI->setLibCallStackSize(LibCallFrameSize);
  }

  // FIXME (note copied from Lanai): This appears to be overallocating.  Needs
  // investigation. Get the number of bytes to allocate from the FrameInfo.
  uint64_t StackSize = MFI.getStackSize();
  uint64_t RealStackSize = StackSize + RVFI->getLibCallStackSize();

  // Early exit if there is no need to allocate on the stack
  if (RealStackSize == 0 && !MFI.adjustsStack())
    return;

  // If the stack pointer has been marked as reserved, then produce an error if
  // the frame requires stack allocation
  if (STI.isRegisterReservedByUser(SPReg))
    MF.getFunction().getContext().diagnose(DiagnosticInfoUnsupported{
        MF.getFunction(), "Stack pointer required, but has been reserved."});

  uint64_t FirstSPAdjustAmount = getFirstSPAdjustAmount(MF);
  // Split the SP adjustment to reduce the offsets of callee saved spill.
  if (FirstSPAdjustAmount) {
    StackSize = FirstSPAdjustAmount;
    RealStackSize = FirstSPAdjustAmount;
  }

  // Allocate space on the stack if necessary.
  adjustReg(MBB, MBBI, DL, SPReg, SPReg, -StackSize, MachineInstr::FrameSetup);

  // Emit ".cfi_def_cfa_offset RealStackSize"
  unsigned CFIIndex = MF.addFrameInst(
      MCCFIInstruction::cfiDefCfaOffset(nullptr, RealStackSize));
  BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  const auto &CSI = MFI.getCalleeSavedInfo();

  // The frame pointer is callee-saved, and code has been generated for us to
  // save it to the stack. We need to skip over the storing of callee-saved
  // registers as the frame pointer must be modified after it has been saved
  // to the stack, not before.
  // FIXME: assumes exactly one instruction is used to save each callee-saved
  // register.
  std::advance(MBBI, getNonLibcallCSI(CSI).size());

  // Iterate over list of callee-saved registers and emit .cfi_offset
  // directives.
  for (const auto &Entry : CSI) {
    int FrameIdx = Entry.getFrameIdx();
    int64_t Offset;
    // Offsets for objects with fixed locations (IE: those saved by libcall) are
    // simply calculated from the frame index.
    if (FrameIdx < 0)
      Offset = FrameIdx * (int64_t) STI.getXLen() / 8;
    else
      Offset = MFI.getObjectOffset(Entry.getFrameIdx()) -
               RVFI->getLibCallStackSize();
    Register Reg = Entry.getReg();
    unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createOffset(
        nullptr, RI->getDwarfRegNum(Reg, true), Offset));
    BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
        .addCFIIndex(CFIIndex);
  }

  // Generate new FP.
  if (hasFP(MF)) {
    if (STI.isRegisterReservedByUser(FPReg))
      MF.getFunction().getContext().diagnose(DiagnosticInfoUnsupported{
          MF.getFunction(), "Frame pointer required, but has been reserved."});

    adjustReg(MBB, MBBI, DL, FPReg, SPReg,
              RealStackSize - RVFI->getVarArgsSaveSize(),
              MachineInstr::FrameSetup);

    // Emit ".cfi_def_cfa $fp, RVFI->getVarArgsSaveSize()"
    unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::cfiDefCfa(
        nullptr, RI->getDwarfRegNum(FPReg, true), RVFI->getVarArgsSaveSize()));
    BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
        .addCFIIndex(CFIIndex);
  }

  // Emit the second SP adjustment after saving callee saved registers.
  if (FirstSPAdjustAmount) {
    uint64_t SecondSPAdjustAmount = MFI.getStackSize() - FirstSPAdjustAmount;
    assert(SecondSPAdjustAmount > 0 &&
           "SecondSPAdjustAmount should be greater than zero");
    adjustReg(MBB, MBBI, DL, SPReg, SPReg, -SecondSPAdjustAmount,
              MachineInstr::FrameSetup);

    // If we are using a frame-pointer, and thus emitted ".cfi_def_cfa fp, 0",
    // don't emit an sp-based .cfi_def_cfa_offset
    if (!hasFP(MF)) {
      // Emit ".cfi_def_cfa_offset StackSize"
      unsigned CFIIndex = MF.addFrameInst(
          MCCFIInstruction::cfiDefCfaOffset(nullptr, MFI.getStackSize()));
      BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
          .addCFIIndex(CFIIndex);
    }
  }

  if (hasFP(MF)) {
    // Realign Stack
    const SISCRegisterInfo *RI = STI.getRegisterInfo();
    if (RI->needsStackRealignment(MF)) {
      Align MaxAlignment = MFI.getMaxAlign();

      const SISCInstrInfo *TII = STI.getInstrInfo();
      if (isInt<12>(-(int)MaxAlignment.value())) {
        BuildMI(MBB, MBBI, DL, TII->get(SISC::ANDI), SPReg)
            .addReg(SPReg)
            .addImm(-(int)MaxAlignment.value());
      } else {
        unsigned ShiftAmount = Log2(MaxAlignment);
        Register VR =
            MF.getRegInfo().createVirtualRegister(&SISC::GPRRegClass);
        BuildMI(MBB, MBBI, DL, TII->get(SISC::SRLI), VR)
            .addReg(SPReg)
            .addImm(ShiftAmount);
        BuildMI(MBB, MBBI, DL, TII->get(SISC::SLLI), SPReg)
            .addReg(VR)
            .addImm(ShiftAmount);
      }
      // FP will be used to restore the frame in the epilogue, so we need
      // another base register BP to record SP after re-alignment. SP will
      // track the current stack after allocating variable sized objects.
      if (hasBP(MF)) {
        // move BP, SP
        BuildMI(MBB, MBBI, DL, TII->get(SISC::ADDI), BPReg)
            .addReg(SPReg)
            .addImm(0);
      }
    }
  }
}

void SISCFrameLowering::emitEpilogue(MachineFunction &MF,
                                      MachineBasicBlock &MBB) const {
  const SISCRegisterInfo *RI = STI.getRegisterInfo();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  auto *RVFI = MF.getInfo<SISCMachineFunctionInfo>();
  Register FPReg = getFPReg(STI);
  Register SPReg = getSPReg(STI);

  // Get the insert location for the epilogue. If there were no terminators in
  // the block, get the last instruction.
  MachineBasicBlock::iterator MBBI = MBB.end();
  DebugLoc DL;
  if (!MBB.empty()) {
    MBBI = MBB.getFirstTerminator();
    if (MBBI == MBB.end())
      MBBI = MBB.getLastNonDebugInstr();
    DL = MBBI->getDebugLoc();

    // If this is not a terminator, the actual insert location should be after the
    // last instruction.
    if (!MBBI->isTerminator())
      MBBI = std::next(MBBI);

    // If callee-saved registers are saved via libcall, place stack adjustment
    // before this call.
    while (MBBI != MBB.begin() &&
           std::prev(MBBI)->getFlag(MachineInstr::FrameDestroy))
      --MBBI;
  }

  const auto &CSI = getNonLibcallCSI(MFI.getCalleeSavedInfo());

  // Skip to before the restores of callee-saved registers
  // FIXME: assumes exactly one instruction is used to restore each
  // callee-saved register.
  auto LastFrameDestroy = MBBI;
  if (!CSI.empty())
    LastFrameDestroy = std::prev(MBBI, CSI.size());

  uint64_t StackSize = MFI.getStackSize();
  uint64_t RealStackSize = StackSize + RVFI->getLibCallStackSize();
  uint64_t FPOffset = RealStackSize - RVFI->getVarArgsSaveSize();

  // Restore the stack pointer using the value of the frame pointer. Only
  // necessary if the stack pointer was modified, meaning the stack size is
  // unknown.
  if (RI->needsStackRealignment(MF) || MFI.hasVarSizedObjects()) {
    assert(hasFP(MF) && "frame pointer should not have been eliminated");
    adjustReg(MBB, LastFrameDestroy, DL, SPReg, FPReg, -FPOffset,
              MachineInstr::FrameDestroy);
  }

  uint64_t FirstSPAdjustAmount = getFirstSPAdjustAmount(MF);
  if (FirstSPAdjustAmount) {
    uint64_t SecondSPAdjustAmount = MFI.getStackSize() - FirstSPAdjustAmount;
    assert(SecondSPAdjustAmount > 0 &&
           "SecondSPAdjustAmount should be greater than zero");

    adjustReg(MBB, LastFrameDestroy, DL, SPReg, SPReg, SecondSPAdjustAmount,
              MachineInstr::FrameDestroy);
  }

  if (FirstSPAdjustAmount)
    StackSize = FirstSPAdjustAmount;

  // Deallocate stack
  adjustReg(MBB, MBBI, DL, SPReg, SPReg, StackSize, MachineInstr::FrameDestroy);
}

int SISCFrameLowering::getFrameIndexReference(const MachineFunction &MF,
                                               int FI,
                                               Register &FrameReg) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo *RI = MF.getSubtarget().getRegisterInfo();
  const auto *RVFI = MF.getInfo<SISCMachineFunctionInfo>();

  // Callee-saved registers should be referenced relative to the stack
  // pointer (positive offset), otherwise use the frame pointer (negative
  // offset).
  const auto &CSI = getNonLibcallCSI(MFI.getCalleeSavedInfo());
  int MinCSFI = 0;
  int MaxCSFI = -1;

  int Offset = MFI.getObjectOffset(FI) - getOffsetOfLocalArea() +
               MFI.getOffsetAdjustment();

  uint64_t FirstSPAdjustAmount = getFirstSPAdjustAmount(MF);

  if (CSI.size()) {
    MinCSFI = CSI[0].getFrameIdx();
    MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  }

  if (FI >= MinCSFI && FI <= MaxCSFI) {
    FrameReg = SISC::X2;

    if (FirstSPAdjustAmount)
      Offset += FirstSPAdjustAmount;
    else
      Offset += MFI.getStackSize();
  } else if (RI->needsStackRealignment(MF) && !MFI.isFixedObjectIndex(FI)) {
    // If the stack was realigned, the frame pointer is set in order to allow
    // SP to be restored, so we need another base register to record the stack
    // after realignment.
    if (hasBP(MF))
      FrameReg = SISCABI::getBPReg();
    else
      FrameReg = SISC::X2;
    Offset += MFI.getStackSize();
    if (FI < 0)
      Offset += RVFI->getLibCallStackSize();
  } else {
    FrameReg = RI->getFrameRegister(MF);
    if (hasFP(MF)) {
      Offset += RVFI->getVarArgsSaveSize();
      if (FI >= 0)
        Offset -= RVFI->getLibCallStackSize();
    } else {
      Offset += MFI.getStackSize();
      if (FI < 0)
        Offset += RVFI->getLibCallStackSize();
    }
  }
  return Offset;
}

void SISCFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                              BitVector &SavedRegs,
                                              RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  // Unconditionally spill RA and FP only if the function uses a frame
  // pointer.
  if (hasFP(MF)) {
    SavedRegs.set(SISC::X1);
    SavedRegs.set(SISC::X8);
  }
  // Mark BP as used if function has dedicated base pointer.
  if (hasBP(MF))
    SavedRegs.set(SISCABI::getBPReg());

  // If interrupt is enabled and there are calls in the handler,
  // unconditionally save all Caller-saved registers and
  // all FP registers, regardless whether they are used.
  MachineFrameInfo &MFI = MF.getFrameInfo();

  if (MF.getFunction().hasFnAttribute("interrupt") && MFI.hasCalls()) {

    static const MCPhysReg CSRegs[] = { SISC::X1,      /* ra */
      SISC::X5, SISC::X6, SISC::X7,                  /* t0-t2 */
      SISC::X10, SISC::X11,                           /* a0-a1, a2-a7 */
      SISC::X12, SISC::X13, SISC::X14, SISC::X15, SISC::X16, SISC::X17,
      SISC::X28, SISC::X29, SISC::X30, SISC::X31, 0 /* t3-t6 */
    };

    for (unsigned i = 0; CSRegs[i]; ++i)
      SavedRegs.set(CSRegs[i]);

    if (MF.getSubtarget<SISCSubtarget>().hasStdExtD() ||
        MF.getSubtarget<SISCSubtarget>().hasStdExtF()) {

      // If interrupt is enabled, this list contains all FP registers.
      const MCPhysReg * Regs = MF.getRegInfo().getCalleeSavedRegs();

      for (unsigned i = 0; Regs[i]; ++i)
        if (SISC::FPR32RegClass.contains(Regs[i]) ||
            SISC::FPR64RegClass.contains(Regs[i]))
          SavedRegs.set(Regs[i]);
    }
  }
}

void SISCFrameLowering::processFunctionBeforeFrameFinalized(
    MachineFunction &MF, RegScavenger *RS) const {
  const TargetRegisterInfo *RegInfo = MF.getSubtarget().getRegisterInfo();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterClass *RC = &SISC::GPRRegClass;
  // estimateStackSize has been observed to under-estimate the final stack
  // size, so give ourselves wiggle-room by checking for stack size
  // representable an 11-bit signed field rather than 12-bits.
  // FIXME: It may be possible to craft a function with a small stack that
  // still needs an emergency spill slot for branch relaxation. This case
  // would currently be missed.
  if (!isInt<11>(MFI.estimateStackSize(MF))) {
    int RegScavFI = MFI.CreateStackObject(RegInfo->getSpillSize(*RC),
                                          RegInfo->getSpillAlign(*RC), false);
    RS->addScavengingFrameIndex(RegScavFI);
  }
}

// Not preserve stack space within prologue for outgoing variables when the
// function contains variable size objects and let eliminateCallFramePseudoInstr
// preserve stack space for it.
bool SISCFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  return !MF.getFrameInfo().hasVarSizedObjects();
}

// Eliminate ADJCALLSTACKDOWN, ADJCALLSTACKUP pseudo instructions.
MachineBasicBlock::iterator SISCFrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {
  Register SPReg = SISC::X2;
  DebugLoc DL = MI->getDebugLoc();

  if (!hasReservedCallFrame(MF)) {
    // If space has not been reserved for a call frame, ADJCALLSTACKDOWN and
    // ADJCALLSTACKUP must be converted to instructions manipulating the stack
    // pointer. This is necessary when there is a variable length stack
    // allocation (e.g. alloca), which means it's not possible to allocate
    // space for outgoing arguments from within the function prologue.
    int64_t Amount = MI->getOperand(0).getImm();

    if (Amount != 0) {
      // Ensure the stack remains aligned after adjustment.
      Amount = alignSPAdjust(Amount);

      if (MI->getOpcode() == SISC::ADJCALLSTACKDOWN)
        Amount = -Amount;

      adjustReg(MBB, MI, DL, SPReg, SPReg, Amount, MachineInstr::NoFlags);
    }
  }

  return MBB.erase(MI);
}

// We would like to split the SP adjustment to reduce prologue/epilogue
// as following instructions. In this way, the offset of the callee saved
// register could fit in a single store.
//   add     sp,sp,-2032
//   sw      ra,2028(sp)
//   sw      s0,2024(sp)
//   sw      s1,2020(sp)
//   sw      s3,2012(sp)
//   sw      s4,2008(sp)
//   add     sp,sp,-64
uint64_t
SISCFrameLowering::getFirstSPAdjustAmount(const MachineFunction &MF) const {
  const auto *RVFI = MF.getInfo<SISCMachineFunctionInfo>();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  uint64_t StackSize = MFI.getStackSize();

  // Disable SplitSPAdjust if save-restore libcall used. The callee saved
  // registers will be pushed by the save-restore libcalls, so we don't have to
  // split the SP adjustment in this case.
  if (RVFI->getLibCallStackSize())
    return 0;

  // Return the FirstSPAdjustAmount if the StackSize can not fit in signed
  // 12-bit and there exists a callee saved register need to be pushed.
  if (!isInt<12>(StackSize) && (CSI.size() > 0)) {
    // FirstSPAdjustAmount is choosed as (2048 - StackAlign)
    // because 2048 will cause sp = sp + 2048 in epilogue split into
    // multi-instructions. The offset smaller than 2048 can fit in signle
    // load/store instruction and we have to stick with the stack alignment.
    // 2048 is 16-byte alignment. The stack alignment for RV32 and RV64 is 16,
    // for RV32E is 4. So (2048 - StackAlign) will satisfy the stack alignment.
    return 2048 - getStackAlign().value();
  }
  return 0;
}

bool SISCFrameLowering::spillCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty())
    return true;

  MachineFunction *MF = MBB.getParent();
  const TargetInstrInfo &TII = *MF->getSubtarget().getInstrInfo();
  DebugLoc DL;
  if (MI != MBB.end() && !MI->isDebugInstr())
    DL = MI->getDebugLoc();

  const char *SpillLibCall = getSpillLibCallName(*MF, CSI);
  if (SpillLibCall) {
    // Add spill libcall via non-callee-saved register t0.
    BuildMI(MBB, MI, DL, TII.get(SISC::PseudoCALLReg), SISC::X5)
        .addExternalSymbol(SpillLibCall, SISCII::MO_CALL)
        .setMIFlag(MachineInstr::FrameSetup);

    // Add registers spilled in libcall as liveins.
    for (auto &CS : CSI)
      MBB.addLiveIn(CS.getReg());
  }

  // Manually spill values not spilled by libcall.
  const auto &NonLibcallCSI = getNonLibcallCSI(CSI);
  for (auto &CS : NonLibcallCSI) {
    // Insert the spill to the stack frame.
    Register Reg = CS.getReg();
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.storeRegToStackSlot(MBB, MI, Reg, true, CS.getFrameIdx(), RC, TRI);
  }

  return true;
}

bool SISCFrameLowering::restoreCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    MutableArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty())
    return true;

  MachineFunction *MF = MBB.getParent();
  const TargetInstrInfo &TII = *MF->getSubtarget().getInstrInfo();
  DebugLoc DL;
  if (MI != MBB.end() && !MI->isDebugInstr())
    DL = MI->getDebugLoc();

  // Manually restore values not restored by libcall. Insert in reverse order.
  // loadRegFromStackSlot can insert multiple instructions.
  const auto &NonLibcallCSI = getNonLibcallCSI(CSI);
  for (auto &CS : reverse(NonLibcallCSI)) {
    Register Reg = CS.getReg();
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.loadRegFromStackSlot(MBB, MI, Reg, CS.getFrameIdx(), RC, TRI);
    assert(MI != MBB.begin() && "loadRegFromStackSlot didn't insert any code!");
  }

  const char *RestoreLibCall = getRestoreLibCallName(*MF, CSI);
  if (RestoreLibCall) {
    // Add restore libcall via tail call.
    MachineBasicBlock::iterator NewMI =
        BuildMI(MBB, MI, DL, TII.get(SISC::PseudoTAIL))
            .addExternalSymbol(RestoreLibCall, SISCII::MO_CALL)
            .setMIFlag(MachineInstr::FrameDestroy);

    // Remove trailing returns, since the terminator is now a tail call to the
    // restore function.
    if (MI != MBB.end() && MI->getOpcode() == SISC::PseudoRET) {
      NewMI->copyImplicitOps(*MF, *MI);
      MI->eraseFromParent();
    }
  }

  return true;
}

bool SISCFrameLowering::canUseAsPrologue(const MachineBasicBlock &MBB) const {
  MachineBasicBlock *TmpMBB = const_cast<MachineBasicBlock *>(&MBB);
  const MachineFunction *MF = MBB.getParent();
  const auto *RVFI = MF->getInfo<SISCMachineFunctionInfo>();

  if (!RVFI->useSaveRestoreLibCalls(*MF))
    return true;

  // Inserting a call to a __sisc_save libcall requires the use of the register
  // t0 (X5) to hold the return address. Therefore if this register is already
  // used we can't insert the call.

  RegScavenger RS;
  RS.enterBasicBlock(*TmpMBB);
  return !RS.isRegUsed(SISC::X5);
}

bool SISCFrameLowering::canUseAsEpilogue(const MachineBasicBlock &MBB) const {
  const MachineFunction *MF = MBB.getParent();
  MachineBasicBlock *TmpMBB = const_cast<MachineBasicBlock *>(&MBB);
  const auto *RVFI = MF->getInfo<SISCMachineFunctionInfo>();

  if (!RVFI->useSaveRestoreLibCalls(*MF))
    return true;

  // Using the __sisc_restore libcalls to restore CSRs requires a tail call.
  // This means if we still need to continue executing code within this function
  // the restore cannot take place in this basic block.

  if (MBB.succ_size() > 1)
    return false;

  MachineBasicBlock *SuccMBB =
      MBB.succ_empty() ? TmpMBB->getFallThrough() : *MBB.succ_begin();

  // Doing a tail call should be safe if there are no successors, because either
  // we have a returning block or the end of the block is unreachable, so the
  // restore will be eliminated regardless.
  if (!SuccMBB)
    return true;

  // The successor can only contain a return, since we would effectively be
  // replacing the successor with our own tail return at the end of our block.
  return SuccMBB->isReturnBlock() && SuccMBB->size() == 1;
}

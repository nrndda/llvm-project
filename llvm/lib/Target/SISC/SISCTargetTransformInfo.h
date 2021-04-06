//===- SISCTargetTransformInfo.h - RISC-V specific TTI ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This file defines a TargetTransformInfo::Concept conforming object specific
/// to the RISC-V target machine. It uses the target's detailed information to
/// provide more precise answers to certain TTI queries, while letting the
/// target independent and default TTI implementations handle the rest.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SISC_SISCTARGETTRANSFORMINFO_H
#define LLVM_LIB_TARGET_SISC_SISCTARGETTRANSFORMINFO_H

#include "SISCSubtarget.h"
#include "SISCTargetMachine.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/BasicTTIImpl.h"
#include "llvm/IR/Function.h"

namespace llvm {

class SISCTTIImpl : public BasicTTIImplBase<SISCTTIImpl> {
  using BaseT = BasicTTIImplBase<SISCTTIImpl>;
  using TTI = TargetTransformInfo;

  friend BaseT;

  const SISCSubtarget *ST;
  const SISCTargetLowering *TLI;

  const SISCSubtarget *getST() const { return ST; }
  const SISCTargetLowering *getTLI() const { return TLI; }

public:
  explicit SISCTTIImpl(const SISCTargetMachine *TM, const Function &F)
      : BaseT(TM, F.getParent()->getDataLayout()), ST(TM->getSubtargetImpl(F)),
        TLI(ST->getTargetLowering()) {}

  int getIntImmCost(const APInt &Imm, Type *Ty, TTI::TargetCostKind CostKind);
  int getIntImmCostInst(unsigned Opcode, unsigned Idx, const APInt &Imm, Type *Ty,
                        TTI::TargetCostKind CostKind);
  int getIntImmCostIntrin(Intrinsic::ID IID, unsigned Idx, const APInt &Imm,
                          Type *Ty, TTI::TargetCostKind CostKind);
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_SISC_SISCTARGETTRANSFORMINFO_H

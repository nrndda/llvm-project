//===-- SISCTargetInfo.cpp - SISC Target Implementation -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/SISCTargetInfo.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target &llvm::getTheSISCTarget() {
  static Target TheSISCTarget;
  return TheSISCTarget;
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeSISCTargetInfo() {
  RegisterTarget<Triple::sisc> X(getTheSISCTarget(), "sisc", "SISC", "SISC");
}

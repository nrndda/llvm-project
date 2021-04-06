//===-- SISCAsmParser.cpp - Parse SISC assembly to MCInst instructions --===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/SISCAsmBackend.h"
#include "MCTargetDesc/SISCMCExpr.h"
#include "MCTargetDesc/SISCMCTargetDesc.h"
#include "MCTargetDesc/SISCTargetStreamer.h"
#include "SISCInstrInfo.h"
#include "TargetInfo/SISCTargetInfo.h"
#include "Utils/SISCBaseInfo.h"
#include "Utils/SISCMatInt.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallBitVector.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/SISCAttributes.h"
#include "llvm/Support/TargetRegistry.h"

#include <limits>

using namespace llvm;

#define DEBUG_TYPE "sisc-asm-parser"

// Include the auto-generated portion of the compress emitter.
#define GEN_COMPRESS_INSTR
#include "SISCGenCompressInstEmitter.inc"

STATISTIC(SISCNumInstrsCompressed,
          "Number of RISC-V Compressed instructions emitted");

namespace {
struct SISCOperand;

struct ParserOptionsSet {
  bool IsPicEnabled;
};

class SISCAsmParser : public MCTargetAsmParser {
  SmallVector<FeatureBitset, 4> FeatureBitStack;

  SmallVector<ParserOptionsSet, 4> ParserOptionsStack;
  ParserOptionsSet ParserOptions;

  SMLoc getLoc() const { return getParser().getTok().getLoc(); }
  bool isRV64() const { return getSTI().hasFeature(SISC::Feature64Bit); }
  bool isRV32E() const { return getSTI().hasFeature(SISC::FeatureRV32E); }

  SISCTargetStreamer &getTargetStreamer() {
    MCTargetStreamer &TS = *getParser().getStreamer().getTargetStreamer();
    return static_cast<SISCTargetStreamer &>(TS);
  }

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;

  bool generateImmOutOfRangeError(OperandVector &Operands, uint64_t ErrorInfo,
                                  int64_t Lower, int64_t Upper, Twine Msg);

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;
  OperandMatchResultTy tryParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                        SMLoc &EndLoc) override;

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override;

  // Helper to actually emit an instruction to the MCStreamer. Also, when
  // possible, compression of the instruction is performed.
  void emitToStreamer(MCStreamer &S, const MCInst &Inst);

  // Helper to emit a combination of LUI, ADDI(W), and SLLI instructions that
  // synthesize the desired immedate value into the destination register.
  void emitLoadImm(Register DestReg, int64_t Value, MCStreamer &Out);

  // Helper to emit a combination of AUIPC and SecondOpcode. Used to implement
  // helpers such as emitLoadLocalAddress and emitLoadAddress.
  void emitAuipcInstPair(MCOperand DestReg, MCOperand TmpReg,
                         const MCExpr *Symbol, SISCMCExpr::VariantKind VKHi,
                         unsigned SecondOpcode, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo instruction "lla" used in PC-rel addressing.
  void emitLoadLocalAddress(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo instruction "la" used in GOT/PC-rel addressing.
  void emitLoadAddress(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo instruction "la.tls.ie" used in initial-exec TLS
  // addressing.
  void emitLoadTLSIEAddress(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo instruction "la.tls.gd" used in global-dynamic TLS
  // addressing.
  void emitLoadTLSGDAddress(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo load/store instruction with a symbol.
  void emitLoadStoreSymbol(MCInst &Inst, unsigned Opcode, SMLoc IDLoc,
                           MCStreamer &Out, bool HasTmpReg);

  // Checks that a PseudoAddTPRel is using x4/tp in its second input operand.
  // Enforcing this using a restricted register class for the second input
  // operand of PseudoAddTPRel results in a poor diagnostic due to the fact
  // 'add' is an overloaded mnemonic.
  bool checkPseudoAddTPRel(MCInst &Inst, OperandVector &Operands);

  // Check instruction constraints.
  bool validateInstruction(MCInst &Inst, OperandVector &Operands);

  /// Helper for processing MC instructions that have been successfully matched
  /// by MatchAndEmitInstruction. Modifications to the emitted instructions,
  /// like the expansion of pseudo instructions (e.g., "li"), can be performed
  /// in this method.
  bool processInstruction(MCInst &Inst, SMLoc IDLoc, OperandVector &Operands,
                          MCStreamer &Out);

// Auto-generated instruction matching functions
#define GET_ASSEMBLER_HEADER
#include "SISCGenAsmMatcher.inc"

  OperandMatchResultTy parseCSRSystemRegister(OperandVector &Operands);
  OperandMatchResultTy parseImmediate(OperandVector &Operands);
  OperandMatchResultTy parseRegister(OperandVector &Operands,
                                     bool AllowParens = false);
  OperandMatchResultTy parseMemOpBaseReg(OperandVector &Operands);
  OperandMatchResultTy parseAtomicMemOp(OperandVector &Operands);
  OperandMatchResultTy parseOperandWithModifier(OperandVector &Operands);
  OperandMatchResultTy parseBareSymbol(OperandVector &Operands);
  OperandMatchResultTy parseCallSymbol(OperandVector &Operands);
  OperandMatchResultTy parsePseudoJumpSymbol(OperandVector &Operands);
  OperandMatchResultTy parseJALOffset(OperandVector &Operands);
  OperandMatchResultTy parseVTypeI(OperandVector &Operands);
  OperandMatchResultTy parseMaskReg(OperandVector &Operands);

  bool parseOperand(OperandVector &Operands, StringRef Mnemonic);

  bool parseDirectiveOption();
  bool parseDirectiveAttribute();

  void setFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (!(getSTI().getFeatureBits()[Feature])) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
    }
  }

  bool getFeatureBits(uint64_t Feature) {
    return getSTI().getFeatureBits()[Feature];
  }

  void clearFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (getSTI().getFeatureBits()[Feature]) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
    }
  }

  void pushFeatureBits() {
    assert(FeatureBitStack.size() == ParserOptionsStack.size() &&
           "These two stacks must be kept synchronized");
    FeatureBitStack.push_back(getSTI().getFeatureBits());
    ParserOptionsStack.push_back(ParserOptions);
  }

  bool popFeatureBits() {
    assert(FeatureBitStack.size() == ParserOptionsStack.size() &&
           "These two stacks must be kept synchronized");
    if (FeatureBitStack.empty())
      return true;

    FeatureBitset FeatureBits = FeatureBitStack.pop_back_val();
    copySTI().setFeatureBits(FeatureBits);
    setAvailableFeatures(ComputeAvailableFeatures(FeatureBits));

    ParserOptions = ParserOptionsStack.pop_back_val();

    return false;
  }

  std::unique_ptr<SISCOperand> defaultMaskRegOp() const;

public:
  enum SISCMatchResultTy {
    Match_Dummy = FIRST_TARGET_MATCH_RESULT_TY,
#define GET_OPERAND_DIAGNOSTIC_TYPES
#include "SISCGenAsmMatcher.inc"
#undef GET_OPERAND_DIAGNOSTIC_TYPES
  };

  static bool classifySymbolRef(const MCExpr *Expr,
                                SISCMCExpr::VariantKind &Kind,
                                int64_t &Addend);

  SISCAsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
                 const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII) {
    Parser.addAliasForDirective(".half", ".2byte");
    Parser.addAliasForDirective(".hword", ".2byte");
    Parser.addAliasForDirective(".word", ".4byte");
    Parser.addAliasForDirective(".dword", ".8byte");
    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));

    auto ABIName = StringRef(Options.ABIName);
    if (ABIName.endswith("f") &&
        !getSTI().getFeatureBits()[SISC::FeatureStdExtF]) {
      errs() << "Hard-float 'f' ABI can't be used for a target that "
                "doesn't support the F instruction set extension (ignoring "
                "target-abi)\n";
    } else if (ABIName.endswith("d") &&
               !getSTI().getFeatureBits()[SISC::FeatureStdExtD]) {
      errs() << "Hard-float 'd' ABI can't be used for a target that "
                "doesn't support the D instruction set extension (ignoring "
                "target-abi)\n";
    }

    const MCObjectFileInfo *MOFI = Parser.getContext().getObjectFileInfo();
    ParserOptions.IsPicEnabled = MOFI->isPositionIndependent();
  }
};

/// SISCOperand - Instances of this class represent a parsed machine
/// instruction
struct SISCOperand : public MCParsedAsmOperand {

  enum class KindTy {
    Token,
    Register,
    Immediate,
    SystemRegister,
    VType,
  } Kind;

  bool IsRV64;

  struct RegOp {
    Register RegNum;
  };

  struct ImmOp {
    const MCExpr *Val;
  };

  struct SysRegOp {
    const char *Data;
    unsigned Length;
    unsigned Encoding;
    // FIXME: Add the Encoding parsed fields as needed for checks,
    // e.g.: read/write or user/supervisor/machine privileges.
  };

  enum class VSEW {
    SEW_8 = 0,
    SEW_16,
    SEW_32,
    SEW_64,
    SEW_128,
    SEW_256,
    SEW_512,
    SEW_1024,
  };

  enum class VLMUL { LMUL_1 = 0, LMUL_2, LMUL_4, LMUL_8 };

  struct VTypeOp {
    VSEW Sew;
    VLMUL Lmul;
    unsigned Encoding;
  };

  SMLoc StartLoc, EndLoc;
  union {
    StringRef Tok;
    RegOp Reg;
    ImmOp Imm;
    struct SysRegOp SysReg;
    struct VTypeOp VType;
  };

  SISCOperand(KindTy K) : MCParsedAsmOperand(), Kind(K) {}

public:
  SISCOperand(const SISCOperand &o) : MCParsedAsmOperand() {
    Kind = o.Kind;
    IsRV64 = o.IsRV64;
    StartLoc = o.StartLoc;
    EndLoc = o.EndLoc;
    switch (Kind) {
    case KindTy::Register:
      Reg = o.Reg;
      break;
    case KindTy::Immediate:
      Imm = o.Imm;
      break;
    case KindTy::Token:
      Tok = o.Tok;
      break;
    case KindTy::SystemRegister:
      SysReg = o.SysReg;
      break;
    case KindTy::VType:
      VType = o.VType;
      break;
    }
  }

  bool isToken() const override { return Kind == KindTy::Token; }
  bool isReg() const override { return Kind == KindTy::Register; }
  bool isV0Reg() const {
    return Kind == KindTy::Register && Reg.RegNum == SISC::V0;
  }
  bool isImm() const override { return Kind == KindTy::Immediate; }
  bool isMem() const override { return false; }
  bool isSystemRegister() const { return Kind == KindTy::SystemRegister; }
  bool isVType() const { return Kind == KindTy::VType; }

  bool isGPR() const {
    return Kind == KindTy::Register &&
           SISCMCRegisterClasses[SISC::GPRRegClassID].contains(Reg.RegNum);
  }

  static bool evaluateConstantImm(const MCExpr *Expr, int64_t &Imm,
                                  SISCMCExpr::VariantKind &VK) {
    if (auto *RE = dyn_cast<SISCMCExpr>(Expr)) {
      VK = RE->getKind();
      return RE->evaluateAsConstant(Imm);
    }

    if (auto CE = dyn_cast<MCConstantExpr>(Expr)) {
      VK = SISCMCExpr::VK_SISC_None;
      Imm = CE->getValue();
      return true;
    }

    return false;
  }

  // True if operand is a symbol with no modifiers, or a constant with no
  // modifiers and isShiftedInt<N-1, 1>(Op).
  template <int N> bool isBareSimmNLsb0() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    bool IsValid;
    if (!IsConstantImm)
      IsValid = SISCAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isShiftedInt<N - 1, 1>(Imm);
    return IsValid && VK == SISCMCExpr::VK_SISC_None;
  }

  // Predicate methods for AsmOperands defined in SISCInstrInfo.td

  bool isBareSymbol() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return SISCAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isCallSymbol() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return SISCAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           (VK == SISCMCExpr::VK_SISC_CALL ||
            VK == SISCMCExpr::VK_SISC_CALL_PLT);
  }

  bool isPseudoJumpSymbol() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return SISCAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           VK == SISCMCExpr::VK_SISC_CALL;
  }

  bool isTPRelAddSymbol() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return SISCAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           VK == SISCMCExpr::VK_SISC_TPREL_ADD;
  }

  bool isCSRSystemRegister() const { return isSystemRegister(); }

  bool isVTypeI() const { return isVType(); }

  /// Return true if the operand is a valid for the fence instruction e.g.
  /// ('iorw').
  bool isFenceArg() const {
    if (!isImm())
      return false;
    const MCExpr *Val = getImm();
    auto *SVal = dyn_cast<MCSymbolRefExpr>(Val);
    if (!SVal || SVal->getKind() != MCSymbolRefExpr::VK_None)
      return false;

    StringRef Str = SVal->getSymbol().getName();
    // Letters must be unique, taken from 'iorw', and in ascending order. This
    // holds as long as each individual character is one of 'iorw' and is
    // greater than the previous character.
    char Prev = '\0';
    for (char c : Str) {
      if (c != 'i' && c != 'o' && c != 'r' && c != 'w')
        return false;
      if (c <= Prev)
        return false;
      Prev = c;
    }
    return true;
  }

  /// Return true if the operand is a valid floating point rounding mode.
  bool isFRMArg() const {
    if (!isImm())
      return false;
    const MCExpr *Val = getImm();
    auto *SVal = dyn_cast<MCSymbolRefExpr>(Val);
    if (!SVal || SVal->getKind() != MCSymbolRefExpr::VK_None)
      return false;

    StringRef Str = SVal->getSymbol().getName();

    return SISCFPRndMode::stringToRoundingMode(Str) != SISCFPRndMode::Invalid;
  }

  bool isImmXLenLI() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (VK == SISCMCExpr::VK_SISC_LO || VK == SISCMCExpr::VK_SISC_PCREL_LO)
      return true;
    // Given only Imm, ensuring that the actually specified constant is either
    // a signed or unsigned 64-bit number is unfortunately impossible.
    return IsConstantImm && VK == SISCMCExpr::VK_SISC_None &&
           (isRV64() || (isInt<32>(Imm) || isUInt<32>(Imm)));
  }

  bool isUImmLog2XLen() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    if (!isImm())
      return false;
    if (!evaluateConstantImm(getImm(), Imm, VK) ||
        VK != SISCMCExpr::VK_SISC_None)
      return false;
    return (isRV64() && isUInt<6>(Imm)) || isUInt<5>(Imm);
  }

  bool isUImmLog2XLenNonZero() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    if (!isImm())
      return false;
    if (!evaluateConstantImm(getImm(), Imm, VK) ||
        VK != SISCMCExpr::VK_SISC_None)
      return false;
    if (Imm == 0)
      return false;
    return (isRV64() && isUInt<6>(Imm)) || isUInt<5>(Imm);
  }

  bool isUImmLog2XLenHalf() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    if (!isImm())
      return false;
    if (!evaluateConstantImm(getImm(), Imm, VK) ||
        VK != SISCMCExpr::VK_SISC_None)
      return false;
    return (isRV64() && isUInt<5>(Imm)) || isUInt<4>(Imm);
  }

  bool isUImm5() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isUInt<5>(Imm) && VK == SISCMCExpr::VK_SISC_None;
  }

  bool isUImm5NonZero() const {
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isUInt<5>(Imm) && (Imm != 0) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isSImm5() const {
    if (!isImm())
      return false;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    int64_t Imm;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isInt<5>(Imm) && VK == SISCMCExpr::VK_SISC_None;
  }

  bool isSImm6() const {
    if (!isImm())
      return false;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    int64_t Imm;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isInt<6>(Imm) &&
	    VK == SISCMCExpr::VK_SISC_None;
  }

  bool isSImm6NonZero() const {
    if (!isImm())
      return false;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    int64_t Imm;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isInt<6>(Imm) && (Imm != 0) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isCLUIImm() const {
    if (!isImm())
      return false;
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && (Imm != 0) &&
           (isUInt<5>(Imm) || (Imm >= 0xfffe0 && Imm <= 0xfffff)) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isUImm7Lsb00() const {
    if (!isImm())
      return false;
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<5, 2>(Imm) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isUImm8Lsb00() const {
    if (!isImm())
      return false;
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<6, 2>(Imm) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isUImm8Lsb000() const {
    if (!isImm())
      return false;
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<5, 3>(Imm) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isSImm9Lsb0() const { return isBareSimmNLsb0<9>(); }

  bool isUImm9Lsb000() const {
    if (!isImm())
      return false;
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<6, 3>(Imm) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isUImm10Lsb00NonZero() const {
    if (!isImm())
      return false;
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<8, 2>(Imm) && (Imm != 0) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isSImm12() const {
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (!IsConstantImm)
      IsValid = SISCAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<12>(Imm);
    return IsValid && ((IsConstantImm && VK == SISCMCExpr::VK_SISC_None) ||
                       VK == SISCMCExpr::VK_SISC_LO ||
                       VK == SISCMCExpr::VK_SISC_PCREL_LO ||
                       VK == SISCMCExpr::VK_SISC_TPREL_LO);
  }

  bool isSImm12Lsb0() const { return isBareSimmNLsb0<12>(); }

  bool isSImm13Lsb0() const { return isBareSimmNLsb0<13>(); }

  bool isSImm10Lsb0000NonZero() const {
    if (!isImm())
      return false;
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && (Imm != 0) && isShiftedInt<6, 4>(Imm) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  bool isUImm20LUI() const {
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (!IsConstantImm) {
      IsValid = SISCAsmParser::classifySymbolRef(getImm(), VK, Imm);
      return IsValid && (VK == SISCMCExpr::VK_SISC_HI ||
                         VK == SISCMCExpr::VK_SISC_TPREL_HI);
    } else {
      return isUInt<20>(Imm) && (VK == SISCMCExpr::VK_SISC_None ||
                                 VK == SISCMCExpr::VK_SISC_HI ||
                                 VK == SISCMCExpr::VK_SISC_TPREL_HI);
    }
  }

  bool isUImm20AUIPC() const {
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (!IsConstantImm) {
      IsValid = SISCAsmParser::classifySymbolRef(getImm(), VK, Imm);
      return IsValid && (VK == SISCMCExpr::VK_SISC_PCREL_HI ||
                         VK == SISCMCExpr::VK_SISC_GOT_HI ||
                         VK == SISCMCExpr::VK_SISC_TLS_GOT_HI ||
                         VK == SISCMCExpr::VK_SISC_TLS_GD_HI);
    } else {
      return isUInt<20>(Imm) && (VK == SISCMCExpr::VK_SISC_None ||
                                 VK == SISCMCExpr::VK_SISC_PCREL_HI ||
                                 VK == SISCMCExpr::VK_SISC_GOT_HI ||
                                 VK == SISCMCExpr::VK_SISC_TLS_GOT_HI ||
                                 VK == SISCMCExpr::VK_SISC_TLS_GD_HI);
    }
  }

  bool isSImm21Lsb0JAL() const { return isBareSimmNLsb0<21>(); }

  bool isImmZero() const {
    if (!isImm())
      return false;
    int64_t Imm;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && (Imm == 0) && VK == SISCMCExpr::VK_SISC_None;
  }

  bool isSImm5Plus1() const {
    if (!isImm())
      return false;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    int64_t Imm;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isInt<5>(Imm - 1) &&
           VK == SISCMCExpr::VK_SISC_None;
  }

  /// getStartLoc - Gets location of the first token of this operand
  SMLoc getStartLoc() const override { return StartLoc; }
  /// getEndLoc - Gets location of the last token of this operand
  SMLoc getEndLoc() const override { return EndLoc; }
  /// True if this operand is for an RV64 instruction
  bool isRV64() const { return IsRV64; }

  unsigned getReg() const override {
    assert(Kind == KindTy::Register && "Invalid type access!");
    return Reg.RegNum.id();
  }

  StringRef getSysReg() const {
    assert(Kind == KindTy::SystemRegister && "Invalid access!");
    return StringRef(SysReg.Data, SysReg.Length);
  }

  const MCExpr *getImm() const {
    assert(Kind == KindTy::Immediate && "Invalid type access!");
    return Imm.Val;
  }

  StringRef getToken() const {
    assert(Kind == KindTy::Token && "Invalid type access!");
    return Tok;
  }

  static StringRef getSEWStr(VSEW Sew) {
    switch (Sew) {
    case VSEW::SEW_8:
      return "e8";
    case VSEW::SEW_16:
      return "e16";
    case VSEW::SEW_32:
      return "e32";
    case VSEW::SEW_64:
      return "e64";
    case VSEW::SEW_128:
      return "e128";
    case VSEW::SEW_256:
      return "e256";
    case VSEW::SEW_512:
      return "e512";
    case VSEW::SEW_1024:
      return "e1024";
    }
    return "";
  }

  static StringRef getLMULStr(VLMUL Lmul) {
    switch (Lmul) {
    case VLMUL::LMUL_1:
      return "m1";
    case VLMUL::LMUL_2:
      return "m2";
    case VLMUL::LMUL_4:
      return "m4";
    case VLMUL::LMUL_8:
      return "m8";
    }
    return "";
  }

  StringRef getVType(SmallString<32> &Buf) const {
    assert(Kind == KindTy::VType && "Invalid access!");
    Buf.append(getSEWStr(VType.Sew));
    Buf.append(",");
    Buf.append(getLMULStr(VType.Lmul));

    return Buf.str();
  }

  void print(raw_ostream &OS) const override {
    switch (Kind) {
    case KindTy::Immediate:
      OS << *getImm();
      break;
    case KindTy::Register:
      OS << "<register x";
      OS << getReg() << ">";
      break;
    case KindTy::Token:
      OS << "'" << getToken() << "'";
      break;
    case KindTy::SystemRegister:
      OS << "<sysreg: " << getSysReg() << '>';
      break;
    case KindTy::VType:
      SmallString<32> VTypeBuf;
      OS << "<vtype: " << getVType(VTypeBuf) << '>';
      break;
    }
  }

  static std::unique_ptr<SISCOperand> createToken(StringRef Str, SMLoc S,
                                                   bool IsRV64) {
    auto Op = std::make_unique<SISCOperand>(KindTy::Token);
    Op->Tok = Str;
    Op->StartLoc = S;
    Op->EndLoc = S;
    Op->IsRV64 = IsRV64;
    return Op;
  }

  static std::unique_ptr<SISCOperand> createReg(unsigned RegNo, SMLoc S,
                                                 SMLoc E, bool IsRV64) {
    auto Op = std::make_unique<SISCOperand>(KindTy::Register);
    Op->Reg.RegNum = RegNo;
    Op->StartLoc = S;
    Op->EndLoc = E;
    Op->IsRV64 = IsRV64;
    return Op;
  }

  static std::unique_ptr<SISCOperand> createImm(const MCExpr *Val, SMLoc S,
                                                 SMLoc E, bool IsRV64) {
    auto Op = std::make_unique<SISCOperand>(KindTy::Immediate);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    Op->IsRV64 = IsRV64;
    return Op;
  }

  static std::unique_ptr<SISCOperand>
  createSysReg(StringRef Str, SMLoc S, unsigned Encoding, bool IsRV64) {
    auto Op = std::make_unique<SISCOperand>(KindTy::SystemRegister);
    Op->SysReg.Data = Str.data();
    Op->SysReg.Length = Str.size();
    Op->SysReg.Encoding = Encoding;
    Op->StartLoc = S;
    Op->IsRV64 = IsRV64;
    return Op;
  }

  static std::unique_ptr<SISCOperand> createVType(APInt Sew, APInt Lmul,
                                                   SMLoc S, bool IsRV64) {
    auto Op = std::make_unique<SISCOperand>(KindTy::VType);
    Sew.ashrInPlace(3);
    unsigned SewLog2 = Sew.logBase2();
    unsigned LmulLog2 = Lmul.logBase2();
    Op->VType.Sew = static_cast<VSEW>(SewLog2);
    Op->VType.Lmul = static_cast<VLMUL>(LmulLog2);
    Op->VType.Encoding = (SewLog2 << 2) | LmulLog2;
    Op->StartLoc = S;
    Op->IsRV64 = IsRV64;
    return Op;
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    assert(Expr && "Expr shouldn't be null!");
    int64_t Imm = 0;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstant = evaluateConstantImm(Expr, Imm, VK);

    if (IsConstant)
      Inst.addOperand(MCOperand::createImm(Imm));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  // Used by the TableGen Code
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }

  void addSImm5Plus1Operands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    int64_t Imm = 0;
    SISCMCExpr::VariantKind VK = SISCMCExpr::VK_SISC_None;
    bool IsConstant = evaluateConstantImm(getImm(), Imm, VK);
    assert(IsConstant && "Expect constant value!");
    (void)IsConstant;
    Inst.addOperand(MCOperand::createImm(Imm - 1));
  }

  void addFenceArgOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    // isFenceArg has validated the operand, meaning this cast is safe
    auto SE = cast<MCSymbolRefExpr>(getImm());

    unsigned Imm = 0;
    for (char c : SE->getSymbol().getName()) {
      switch (c) {
      default:
        llvm_unreachable("FenceArg must contain only [iorw]");
      case 'i': Imm |= SISCFenceField::I; break;
      case 'o': Imm |= SISCFenceField::O; break;
      case 'r': Imm |= SISCFenceField::R; break;
      case 'w': Imm |= SISCFenceField::W; break;
      }
    }
    Inst.addOperand(MCOperand::createImm(Imm));
  }

  void addCSRSystemRegisterOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createImm(SysReg.Encoding));
  }

  void addVTypeIOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createImm(VType.Encoding));
  }

  // Returns the rounding mode represented by this SISCOperand. Should only
  // be called after checking isFRMArg.
  SISCFPRndMode::RoundingMode getRoundingMode() const {
    // isFRMArg has validated the operand, meaning this cast is safe.
    auto SE = cast<MCSymbolRefExpr>(getImm());
    SISCFPRndMode::RoundingMode FRM =
        SISCFPRndMode::stringToRoundingMode(SE->getSymbol().getName());
    assert(FRM != SISCFPRndMode::Invalid && "Invalid rounding mode");
    return FRM;
  }

  void addFRMArgOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createImm(getRoundingMode()));
  }
};
} // end anonymous namespace.

#define GET_REGISTER_MATCHER
#define GET_SUBTARGET_FEATURE_NAME
#define GET_MATCHER_IMPLEMENTATION
#define GET_MNEMONIC_SPELL_CHECKER
#include "SISCGenAsmMatcher.inc"

static Register convertFPR64ToFPR32(Register Reg) {
  assert(Reg >= SISC::F0_D && Reg <= SISC::F31_D && "Invalid register");
  return Reg - SISC::F0_D + SISC::F0_F;
}

unsigned SISCAsmParser::validateTargetOperandClass(MCParsedAsmOperand &AsmOp,
                                                    unsigned Kind) {
  SISCOperand &Op = static_cast<SISCOperand &>(AsmOp);
  if (!Op.isReg())
    return Match_InvalidOperand;

  Register Reg = Op.getReg();
  bool IsRegFPR64 =
      SISCMCRegisterClasses[SISC::FPR64RegClassID].contains(Reg);
  bool IsRegFPR64C =
      SISCMCRegisterClasses[SISC::FPR64CRegClassID].contains(Reg);

  // As the parser couldn't differentiate an FPR32 from an FPR64, coerce the
  // register from FPR64 to FPR32 or FPR64C to FPR32C if necessary.
  if ((IsRegFPR64 && Kind == MCK_FPR32) ||
      (IsRegFPR64C && Kind == MCK_FPR32C)) {
    Op.Reg.RegNum = convertFPR64ToFPR32(Reg);
    return Match_Success;
  }
  return Match_InvalidOperand;
}

bool SISCAsmParser::generateImmOutOfRangeError(
    OperandVector &Operands, uint64_t ErrorInfo, int64_t Lower, int64_t Upper,
    Twine Msg = "immediate must be an integer in the range") {
  SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
  return Error(ErrorLoc, Msg + " [" + Twine(Lower) + ", " + Twine(Upper) + "]");
}

static std::string SISCMnemonicSpellCheck(StringRef S,
                                          const FeatureBitset &FBS,
                                          unsigned VariantID = 0);

bool SISCAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                             OperandVector &Operands,
                                             MCStreamer &Out,
                                             uint64_t &ErrorInfo,
                                             bool MatchingInlineAsm) {
  MCInst Inst;
  FeatureBitset MissingFeatures;

  auto Result =
    MatchInstructionImpl(Operands, Inst, ErrorInfo, MissingFeatures,
                         MatchingInlineAsm);
  switch (Result) {
  default:
    break;
  case Match_Success:
    if (validateInstruction(Inst, Operands))
      return true;
    return processInstruction(Inst, IDLoc, Operands, Out);
  case Match_MissingFeature: {
    assert(MissingFeatures.any() && "Unknown missing features!");
    bool FirstFeature = true;
    std::string Msg = "instruction requires the following:";
    for (unsigned i = 0, e = MissingFeatures.size(); i != e; ++i) {
      if (MissingFeatures[i]) {
        Msg += FirstFeature ? " " : ", ";
        Msg += getSubtargetFeatureName(i);
        FirstFeature = false;
      }
    }
    return Error(IDLoc, Msg);
  }
  case Match_MnemonicFail: {
    FeatureBitset FBS = ComputeAvailableFeatures(getSTI().getFeatureBits());
    std::string Suggestion = SISCMnemonicSpellCheck(
      ((SISCOperand &)*Operands[0]).getToken(), FBS);
    return Error(IDLoc, "unrecognized instruction mnemonic" + Suggestion);
  }
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");

      ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }
    return Error(ErrorLoc, "invalid operand for instruction");
  }
  }

  // Handle the case when the error message is of specific type
  // other than the generic Match_InvalidOperand, and the
  // corresponding operand is missing.
  if (Result > FIRST_TARGET_MATCH_RESULT_TY) {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U && ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");
  }

  switch(Result) {
  default:
    break;
  case Match_InvalidImmXLenLI:
    if (isRV64()) {
      SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
      return Error(ErrorLoc, "operand must be a constant 64-bit integer");
    }
    return generateImmOutOfRangeError(Operands, ErrorInfo,
                                      std::numeric_limits<int32_t>::min(),
                                      std::numeric_limits<uint32_t>::max());
  case Match_InvalidImmZero: {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "immediate must be zero");
  }
  case Match_InvalidUImmLog2XLen:
    if (isRV64())
      return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 6) - 1);
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 5) - 1);
  case Match_InvalidUImmLog2XLenNonZero:
    if (isRV64())
      return generateImmOutOfRangeError(Operands, ErrorInfo, 1, (1 << 6) - 1);
    return generateImmOutOfRangeError(Operands, ErrorInfo, 1, (1 << 5) - 1);
  case Match_InvalidUImmLog2XLenHalf:
    if (isRV64())
      return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 5) - 1);
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 4) - 1);
  case Match_InvalidUImm5:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 5) - 1);
  case Match_InvalidSImm6:
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 5),
                                      (1 << 5) - 1);
  case Match_InvalidSImm6NonZero:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 5), (1 << 5) - 1,
        "immediate must be non-zero in the range");
  case Match_InvalidCLUIImm:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 1, (1 << 5) - 1,
        "immediate must be in [0xfffe0, 0xfffff] or");
  case Match_InvalidUImm7Lsb00:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 7) - 4,
        "immediate must be a multiple of 4 bytes in the range");
  case Match_InvalidUImm8Lsb00:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 8) - 4,
        "immediate must be a multiple of 4 bytes in the range");
  case Match_InvalidUImm8Lsb000:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 8) - 8,
        "immediate must be a multiple of 8 bytes in the range");
  case Match_InvalidSImm9Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 8), (1 << 8) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidUImm9Lsb000:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 9) - 8,
        "immediate must be a multiple of 8 bytes in the range");
  case Match_InvalidUImm10Lsb00NonZero:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 4, (1 << 10) - 4,
        "immediate must be a multiple of 4 bytes in the range");
  case Match_InvalidSImm10Lsb0000NonZero:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 9), (1 << 9) - 16,
        "immediate must be a multiple of 16 bytes and non-zero in the range");
  case Match_InvalidSImm12:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 11), (1 << 11) - 1,
        "operand must be a symbol with %lo/%pcrel_lo/%tprel_lo modifier or an "
        "integer in the range");
  case Match_InvalidSImm12Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 11), (1 << 11) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidSImm13Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 12), (1 << 12) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidUImm20LUI:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 20) - 1,
                                      "operand must be a symbol with "
                                      "%hi/%tprel_hi modifier or an integer in "
                                      "the range");
  case Match_InvalidUImm20AUIPC:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 20) - 1,
        "operand must be a symbol with a "
        "%pcrel_hi/%got_pcrel_hi/%tls_ie_pcrel_hi/%tls_gd_pcrel_hi modifier or "
        "an integer in the range");
  case Match_InvalidSImm21Lsb0JAL:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 20), (1 << 20) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidCSRSystemRegister: {
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 12) - 1,
                                      "operand must be a valid system register "
                                      "name or an integer in the range");
  }
  case Match_InvalidFenceArg: {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(
        ErrorLoc,
        "operand must be formed of letters selected in-order from 'iorw'");
  }
  case Match_InvalidFRMArg: {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(
        ErrorLoc,
        "operand must be a valid floating point rounding mode mnemonic");
  }
  case Match_InvalidBareSymbol: {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a bare symbol name");
  }
  case Match_InvalidPseudoJumpSymbol: {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a valid jump target");
  }
  case Match_InvalidCallSymbol: {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a bare symbol name");
  }
  case Match_InvalidTPRelAddSymbol: {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a symbol with %tprel_add modifier");
  }
  case Match_InvalidVTypeI: {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc,
                 "operand must be e[8|16|32|64|128|256|512|1024],m[1|2|4|8]");
  }
  case Match_InvalidVMaskRegister: {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be v0.t");
  }
  case Match_InvalidSImm5Plus1: {
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 4) + 1,
                                      (1 << 4),
                                      "immediate must be in the range");
  }
  }

  llvm_unreachable("Unknown match type detected!");
}

// Attempts to match Name as a register (either using the default name or
// alternative ABI names), setting RegNo to the matching register. Upon
// failure, returns true and sets RegNo to 0. If IsRV32E then registers
// x16-x31 will be rejected.
static bool matchRegisterNameHelper(bool IsRV32E, Register &RegNo,
                                    StringRef Name) {
  RegNo = MatchRegisterName(Name);
  // The 32- and 64-bit FPRs have the same asm name. Check that the initial
  // match always matches the 64-bit variant, and not the 32-bit one.
  assert(!(RegNo >= SISC::F0_F && RegNo <= SISC::F31_F));
  // The default FPR register class is based on the tablegen enum ordering.
  static_assert(SISC::F0_D < SISC::F0_F, "FPR matching must be updated");
  if (RegNo == SISC::NoRegister)
    RegNo = MatchRegisterAltName(Name);
  if (IsRV32E && RegNo >= SISC::X16 && RegNo <= SISC::X31)
    RegNo = SISC::NoRegister;
  return RegNo == SISC::NoRegister;
}

bool SISCAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                   SMLoc &EndLoc) {
  if (tryParseRegister(RegNo, StartLoc, EndLoc) != MatchOperand_Success)
    return Error(StartLoc, "invalid register name");
  return false;
}

OperandMatchResultTy SISCAsmParser::tryParseRegister(unsigned &RegNo,
                                                      SMLoc &StartLoc,
                                                      SMLoc &EndLoc) {
  const AsmToken &Tok = getParser().getTok();
  StartLoc = Tok.getLoc();
  EndLoc = Tok.getEndLoc();
  RegNo = 0;
  StringRef Name = getLexer().getTok().getIdentifier();

  if (matchRegisterNameHelper(isRV32E(), (Register &)RegNo, Name))
    return MatchOperand_NoMatch;

  getParser().Lex(); // Eat identifier token.
  return MatchOperand_Success;
}

OperandMatchResultTy SISCAsmParser::parseRegister(OperandVector &Operands,
                                                   bool AllowParens) {
  SMLoc FirstS = getLoc();
  bool HadParens = false;
  AsmToken LParen;

  // If this is an LParen and a parenthesised register name is allowed, parse it
  // atomically.
  if (AllowParens && getLexer().is(AsmToken::LParen)) {
    AsmToken Buf[2];
    size_t ReadCount = getLexer().peekTokens(Buf);
    if (ReadCount == 2 && Buf[1].getKind() == AsmToken::RParen) {
      HadParens = true;
      LParen = getParser().getTok();
      getParser().Lex(); // Eat '('
    }
  }

  switch (getLexer().getKind()) {
  default:
    if (HadParens)
      getLexer().UnLex(LParen);
    return MatchOperand_NoMatch;
  case AsmToken::Identifier:
    StringRef Name = getLexer().getTok().getIdentifier();
    Register RegNo;
    matchRegisterNameHelper(isRV32E(), RegNo, Name);

    if (RegNo == SISC::NoRegister) {
      if (HadParens)
        getLexer().UnLex(LParen);
      return MatchOperand_NoMatch;
    }
    if (HadParens)
      Operands.push_back(SISCOperand::createToken("(", FirstS, isRV64()));
    SMLoc S = getLoc();
    SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
    getLexer().Lex();
    Operands.push_back(SISCOperand::createReg(RegNo, S, E, isRV64()));
  }

  if (HadParens) {
    getParser().Lex(); // Eat ')'
    Operands.push_back(SISCOperand::createToken(")", getLoc(), isRV64()));
  }

  return MatchOperand_Success;
}

OperandMatchResultTy
SISCAsmParser::parseCSRSystemRegister(OperandVector &Operands) {
  SMLoc S = getLoc();
  const MCExpr *Res;

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Exclaim:
  case AsmToken::Tilde:
  case AsmToken::Integer:
  case AsmToken::String: {
    if (getParser().parseExpression(Res))
      return MatchOperand_ParseFail;

    auto *CE = dyn_cast<MCConstantExpr>(Res);
    if (CE) {
      int64_t Imm = CE->getValue();
      if (isUInt<12>(Imm)) {
        auto SysReg = SISCSysReg::lookupSysRegByEncoding(Imm);
        // Accept an immediate representing a named or un-named Sys Reg
        // if the range is valid, regardless of the required features.
        Operands.push_back(SISCOperand::createSysReg(
            SysReg ? SysReg->Name : "", S, Imm, isRV64()));
        return MatchOperand_Success;
      }
    }

    Twine Msg = "immediate must be an integer in the range";
    Error(S, Msg + " [" + Twine(0) + ", " + Twine((1 << 12) - 1) + "]");
    return MatchOperand_ParseFail;
  }
  case AsmToken::Identifier: {
    StringRef Identifier;
    if (getParser().parseIdentifier(Identifier))
      return MatchOperand_ParseFail;

    auto SysReg = SISCSysReg::lookupSysRegByName(Identifier);
    if (!SysReg)
      SysReg = SISCSysReg::lookupSysRegByAltName(Identifier);
    // Accept a named Sys Reg if the required features are present.
    if (SysReg) {
      if (!SysReg->haveRequiredFeatures(getSTI().getFeatureBits())) {
        Error(S, "system register use requires an option to be enabled");
        return MatchOperand_ParseFail;
      }
      Operands.push_back(SISCOperand::createSysReg(
          Identifier, S, SysReg->Encoding, isRV64()));
      return MatchOperand_Success;
    }

    Twine Msg = "operand must be a valid system register name "
                "or an integer in the range";
    Error(S, Msg + " [" + Twine(0) + ", " + Twine((1 << 12) - 1) + "]");
    return MatchOperand_ParseFail;
  }
  case AsmToken::Percent: {
    // Discard operand with modifier.
    Twine Msg = "immediate must be an integer in the range";
    Error(S, Msg + " [" + Twine(0) + ", " + Twine((1 << 12) - 1) + "]");
    return MatchOperand_ParseFail;
  }
  }

  return MatchOperand_NoMatch;
}

OperandMatchResultTy SISCAsmParser::parseImmediate(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Dot:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Exclaim:
  case AsmToken::Tilde:
  case AsmToken::Integer:
  case AsmToken::String:
  case AsmToken::Identifier:
    if (getParser().parseExpression(Res))
      return MatchOperand_ParseFail;
    break;
  case AsmToken::Percent:
    return parseOperandWithModifier(Operands);
  }

  Operands.push_back(SISCOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy
SISCAsmParser::parseOperandWithModifier(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);

  if (getLexer().getKind() != AsmToken::Percent) {
    Error(getLoc(), "expected '%' for operand modifier");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat '%'

  if (getLexer().getKind() != AsmToken::Identifier) {
    Error(getLoc(), "expected valid identifier for operand modifier");
    return MatchOperand_ParseFail;
  }
  StringRef Identifier = getParser().getTok().getIdentifier();
  SISCMCExpr::VariantKind VK = SISCMCExpr::getVariantKindForName(Identifier);
  if (VK == SISCMCExpr::VK_SISC_Invalid) {
    Error(getLoc(), "unrecognized operand modifier");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat the identifier
  if (getLexer().getKind() != AsmToken::LParen) {
    Error(getLoc(), "expected '('");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat '('

  const MCExpr *SubExpr;
  if (getParser().parseParenExpression(SubExpr, E)) {
    return MatchOperand_ParseFail;
  }

  const MCExpr *ModExpr = SISCMCExpr::create(SubExpr, VK, getContext());
  Operands.push_back(SISCOperand::createImm(ModExpr, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy SISCAsmParser::parseBareSymbol(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  if (getLexer().getKind() != AsmToken::Identifier)
    return MatchOperand_NoMatch;

  StringRef Identifier;
  AsmToken Tok = getLexer().getTok();

  if (getParser().parseIdentifier(Identifier))
    return MatchOperand_ParseFail;

  if (Identifier.consume_back("@plt")) {
    Error(getLoc(), "'@plt' operand not valid for instruction");
    return MatchOperand_ParseFail;
  }

  MCSymbol *Sym = getContext().getOrCreateSymbol(Identifier);

  if (Sym->isVariable()) {
    const MCExpr *V = Sym->getVariableValue(/*SetUsed=*/false);
    if (!isa<MCSymbolRefExpr>(V)) {
      getLexer().UnLex(Tok); // Put back if it's not a bare symbol.
      return MatchOperand_NoMatch;
    }
    Res = V;
  } else
    Res = MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());

  MCBinaryExpr::Opcode Opcode;
  switch (getLexer().getKind()) {
  default:
    Operands.push_back(SISCOperand::createImm(Res, S, E, isRV64()));
    return MatchOperand_Success;
  case AsmToken::Plus:
    Opcode = MCBinaryExpr::Add;
    break;
  case AsmToken::Minus:
    Opcode = MCBinaryExpr::Sub;
    break;
  }

  const MCExpr *Expr;
  if (getParser().parseExpression(Expr))
    return MatchOperand_ParseFail;
  Res = MCBinaryExpr::create(Opcode, Res, Expr, getContext());
  Operands.push_back(SISCOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy SISCAsmParser::parseCallSymbol(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  if (getLexer().getKind() != AsmToken::Identifier)
    return MatchOperand_NoMatch;

  // Avoid parsing the register in `call rd, foo` as a call symbol.
  if (getLexer().peekTok().getKind() != AsmToken::EndOfStatement)
    return MatchOperand_NoMatch;

  StringRef Identifier;
  if (getParser().parseIdentifier(Identifier))
    return MatchOperand_ParseFail;

  SISCMCExpr::VariantKind Kind = SISCMCExpr::VK_SISC_CALL;
  if (Identifier.consume_back("@plt"))
    Kind = SISCMCExpr::VK_SISC_CALL_PLT;

  MCSymbol *Sym = getContext().getOrCreateSymbol(Identifier);
  Res = MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
  Res = SISCMCExpr::create(Res, Kind, getContext());
  Operands.push_back(SISCOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy
SISCAsmParser::parsePseudoJumpSymbol(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  if (getParser().parseExpression(Res))
    return MatchOperand_ParseFail;

  if (Res->getKind() != MCExpr::ExprKind::SymbolRef ||
      cast<MCSymbolRefExpr>(Res)->getKind() ==
          MCSymbolRefExpr::VariantKind::VK_PLT) {
    Error(S, "operand must be a valid jump target");
    return MatchOperand_ParseFail;
  }

  Res = SISCMCExpr::create(Res, SISCMCExpr::VK_SISC_CALL, getContext());
  Operands.push_back(SISCOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy SISCAsmParser::parseJALOffset(OperandVector &Operands) {
  // Parsing jal operands is fiddly due to the `jal foo` and `jal ra, foo`
  // both being acceptable forms. When parsing `jal ra, foo` this function
  // will be called for the `ra` register operand in an attempt to match the
  // single-operand alias. parseJALOffset must fail for this case. It would
  // seem logical to try parse the operand using parseImmediate and return
  // NoMatch if the next token is a comma (meaning we must be parsing a jal in
  // the second form rather than the first). We can't do this as there's no
  // way of rewinding the lexer state. Instead, return NoMatch if this operand
  // is an identifier and is followed by a comma.
  if (getLexer().is(AsmToken::Identifier) &&
      getLexer().peekTok().is(AsmToken::Comma))
    return MatchOperand_NoMatch;

  return parseImmediate(Operands);
}

OperandMatchResultTy SISCAsmParser::parseVTypeI(OperandVector &Operands) {
  SMLoc S = getLoc();
  if (getLexer().getKind() != AsmToken::Identifier)
    return MatchOperand_NoMatch;

  // Parse "e8,m1"
  StringRef Name = getLexer().getTok().getIdentifier();
  if (!Name.consume_front("e"))
    return MatchOperand_NoMatch;
  APInt Sew(16, Name, 10);
  if (Sew != 8 && Sew != 16 && Sew != 32 && Sew != 64 && Sew != 128 &&
      Sew != 256 && Sew != 512 && Sew != 1024)
    return MatchOperand_NoMatch;
  getLexer().Lex();

  if (getLexer().getKind() == AsmToken::EndOfStatement) {
    Operands.push_back(
        SISCOperand::createVType(Sew, APInt(16, 1), S, isRV64()));

    return MatchOperand_Success;
  }

  if (!getLexer().is(AsmToken::Comma))
    return MatchOperand_NoMatch;
  getLexer().Lex();

  Name = getLexer().getTok().getIdentifier();
  if (!Name.consume_front("m"))
    return MatchOperand_NoMatch;
  APInt Lmul(16, Name, 10);
  if (Lmul != 1 && Lmul != 2 && Lmul != 4 && Lmul != 8)
    return MatchOperand_NoMatch;
  getLexer().Lex();

  if (getLexer().getKind() != AsmToken::EndOfStatement)
    return MatchOperand_NoMatch;

  Operands.push_back(SISCOperand::createVType(Sew, Lmul, S, isRV64()));

  return MatchOperand_Success;
}

OperandMatchResultTy SISCAsmParser::parseMaskReg(OperandVector &Operands) {
  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::Identifier:
    StringRef Name = getLexer().getTok().getIdentifier();
    if (!Name.consume_back(".t")) {
      Error(getLoc(), "expected '.t' suffix");
      return MatchOperand_ParseFail;
    }
    Register RegNo;
    matchRegisterNameHelper(isRV32E(), RegNo, Name);

    if (RegNo == SISC::NoRegister)
      return MatchOperand_NoMatch;
    if (RegNo != SISC::V0)
      return MatchOperand_NoMatch;
    SMLoc S = getLoc();
    SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
    getLexer().Lex();
    Operands.push_back(SISCOperand::createReg(RegNo, S, E, isRV64()));
  }

  return MatchOperand_Success;
}

OperandMatchResultTy
SISCAsmParser::parseMemOpBaseReg(OperandVector &Operands) {
  if (getLexer().isNot(AsmToken::LParen)) {
    Error(getLoc(), "expected '('");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat '('
  Operands.push_back(SISCOperand::createToken("(", getLoc(), isRV64()));

  if (parseRegister(Operands) != MatchOperand_Success) {
    Error(getLoc(), "expected register");
    return MatchOperand_ParseFail;
  }

  if (getLexer().isNot(AsmToken::RParen)) {
    Error(getLoc(), "expected ')'");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat ')'
  Operands.push_back(SISCOperand::createToken(")", getLoc(), isRV64()));

  return MatchOperand_Success;
}

OperandMatchResultTy SISCAsmParser::parseAtomicMemOp(OperandVector &Operands) {
  // Atomic operations such as lr.w, sc.w, and amo*.w accept a "memory operand"
  // as one of their register operands, such as `(a0)`. This just denotes that
  // the register (in this case `a0`) contains a memory address.
  //
  // Normally, we would be able to parse these by putting the parens into the
  // instruction string. However, GNU as also accepts a zero-offset memory
  // operand (such as `0(a0)`), and ignores the 0. Normally this would be parsed
  // with parseImmediate followed by parseMemOpBaseReg, but these instructions
  // do not accept an immediate operand, and we do not want to add a "dummy"
  // operand that is silently dropped.
  //
  // Instead, we use this custom parser. This will: allow (and discard) an
  // offset if it is zero; require (and discard) parentheses; and add only the
  // parsed register operand to `Operands`.
  //
  // These operands are printed with SISCInstPrinter::printAtomicMemOp, which
  // will only print the register surrounded by parentheses (which GNU as also
  // uses as its canonical representation for these operands).
  std::unique_ptr<SISCOperand> OptionalImmOp;

  if (getLexer().isNot(AsmToken::LParen)) {
    // Parse an Integer token. We do not accept arbritrary constant expressions
    // in the offset field (because they may include parens, which complicates
    // parsing a lot).
    int64_t ImmVal;
    SMLoc ImmStart = getLoc();
    if (getParser().parseIntToken(ImmVal,
                                  "expected '(' or optional integer offset"))
      return MatchOperand_ParseFail;

    // Create a SISCOperand for checking later (so the error messages are
    // nicer), but we don't add it to Operands.
    SMLoc ImmEnd = getLoc();
    OptionalImmOp =
        SISCOperand::createImm(MCConstantExpr::create(ImmVal, getContext()),
                                ImmStart, ImmEnd, isRV64());
  }

  if (getLexer().isNot(AsmToken::LParen)) {
    Error(getLoc(), OptionalImmOp ? "expected '(' after optional integer offset"
                                  : "expected '(' or optional integer offset");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat '('

  if (parseRegister(Operands) != MatchOperand_Success) {
    Error(getLoc(), "expected register");
    return MatchOperand_ParseFail;
  }

  if (getLexer().isNot(AsmToken::RParen)) {
    Error(getLoc(), "expected ')'");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat ')'

  // Deferred Handling of non-zero offsets. This makes the error messages nicer.
  if (OptionalImmOp && !OptionalImmOp->isImmZero()) {
    Error(OptionalImmOp->getStartLoc(), "optional integer offset must be 0",
          SMRange(OptionalImmOp->getStartLoc(), OptionalImmOp->getEndLoc()));
    return MatchOperand_ParseFail;
  }

  return MatchOperand_Success;
}

/// Looks at a token type and creates the relevant operand from this
/// information, adding to Operands. If operand was parsed, returns false, else
/// true.
bool SISCAsmParser::parseOperand(OperandVector &Operands, StringRef Mnemonic) {
  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  OperandMatchResultTy Result =
      MatchOperandParserImpl(Operands, Mnemonic, /*ParseForAllFeatures=*/true);
  if (Result == MatchOperand_Success)
    return false;
  if (Result == MatchOperand_ParseFail)
    return true;

  // Attempt to parse token as a register.
  if (parseRegister(Operands, true) == MatchOperand_Success)
    return false;

  // Attempt to parse token as an immediate
  if (parseImmediate(Operands) == MatchOperand_Success) {
    // Parse memory base register if present
    if (getLexer().is(AsmToken::LParen))
      return parseMemOpBaseReg(Operands) != MatchOperand_Success;
    return false;
  }

  // Finally we have exhausted all options and must declare defeat.
  Error(getLoc(), "unknown operand");
  return true;
}

bool SISCAsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                      StringRef Name, SMLoc NameLoc,
                                      OperandVector &Operands) {
  // Ensure that if the instruction occurs when relaxation is enabled,
  // relocations are forced for the file. Ideally this would be done when there
  // is enough information to reliably determine if the instruction itself may
  // cause relaxations. Unfortunately instruction processing stage occurs in the
  // same pass as relocation emission, so it's too late to set a 'sticky bit'
  // for the entire file.
  if (getSTI().getFeatureBits()[SISC::FeatureRelax]) {
    auto *Assembler = getTargetStreamer().getStreamer().getAssemblerPtr();
    if (Assembler != nullptr) {
      SISCAsmBackend &MAB =
          static_cast<SISCAsmBackend &>(Assembler->getBackend());
      MAB.setForceRelocs();
    }
  }

  // First operand is token for instruction
  Operands.push_back(SISCOperand::createToken(Name, NameLoc, isRV64()));

  // If there are no more operands, then finish
  if (getLexer().is(AsmToken::EndOfStatement))
    return false;

  // Parse first operand
  if (parseOperand(Operands, Name))
    return true;

  // Parse until end of statement, consuming commas between operands
  unsigned OperandIdx = 1;
  while (getLexer().is(AsmToken::Comma)) {
    // Consume comma token
    getLexer().Lex();

    // Parse next operand
    if (parseOperand(Operands, Name))
      return true;

    ++OperandIdx;
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    getParser().eatToEndOfStatement();
    return Error(Loc, "unexpected token");
  }

  getParser().Lex(); // Consume the EndOfStatement.
  return false;
}

bool SISCAsmParser::classifySymbolRef(const MCExpr *Expr,
                                       SISCMCExpr::VariantKind &Kind,
                                       int64_t &Addend) {
  Kind = SISCMCExpr::VK_SISC_None;
  Addend = 0;

  if (const SISCMCExpr *RE = dyn_cast<SISCMCExpr>(Expr)) {
    Kind = RE->getKind();
    Expr = RE->getSubExpr();
  }

  // It's a simple symbol reference or constant with no addend.
  if (isa<MCConstantExpr>(Expr) || isa<MCSymbolRefExpr>(Expr))
    return true;

  const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(Expr);
  if (!BE)
    return false;

  if (!isa<MCSymbolRefExpr>(BE->getLHS()))
    return false;

  if (BE->getOpcode() != MCBinaryExpr::Add &&
      BE->getOpcode() != MCBinaryExpr::Sub)
    return false;

  // We are able to support the subtraction of two symbol references
  if (BE->getOpcode() == MCBinaryExpr::Sub &&
      isa<MCSymbolRefExpr>(BE->getRHS()))
    return true;

  // See if the addend is a constant, otherwise there's more going
  // on here than we can deal with.
  auto AddendExpr = dyn_cast<MCConstantExpr>(BE->getRHS());
  if (!AddendExpr)
    return false;

  Addend = AddendExpr->getValue();
  if (BE->getOpcode() == MCBinaryExpr::Sub)
    Addend = -Addend;

  // It's some symbol reference + a constant addend
  return Kind != SISCMCExpr::VK_SISC_Invalid;
}

bool SISCAsmParser::ParseDirective(AsmToken DirectiveID) {
  // This returns false if this function recognizes the directive
  // regardless of whether it is successfully handles or reports an
  // error. Otherwise it returns true to give the generic parser a
  // chance at recognizing it.
  StringRef IDVal = DirectiveID.getString();

  if (IDVal == ".option")
    return parseDirectiveOption();
  else if (IDVal == ".attribute")
    return parseDirectiveAttribute();

  return true;
}

bool SISCAsmParser::parseDirectiveOption() {
  MCAsmParser &Parser = getParser();
  // Get the option token.
  AsmToken Tok = Parser.getTok();
  // At the moment only identifiers are supported.
  if (Tok.isNot(AsmToken::Identifier))
    return Error(Parser.getTok().getLoc(),
                 "unexpected token, expected identifier");

  StringRef Option = Tok.getIdentifier();

  if (Option == "push") {
    getTargetStreamer().emitDirectiveOptionPush();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    pushFeatureBits();
    return false;
  }

  if (Option == "pop") {
    SMLoc StartLoc = Parser.getTok().getLoc();
    getTargetStreamer().emitDirectiveOptionPop();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    if (popFeatureBits())
      return Error(StartLoc, ".option pop with no .option push");

    return false;
  }

  if (Option == "rvc") {
    getTargetStreamer().emitDirectiveOptionRVC();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    setFeatureBits(SISC::FeatureStdExtC, "c");
    return false;
  }

  if (Option == "norvc") {
    getTargetStreamer().emitDirectiveOptionNoRVC();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    clearFeatureBits(SISC::FeatureStdExtC, "c");
    return false;
  }

  if (Option == "pic") {
    getTargetStreamer().emitDirectiveOptionPIC();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    ParserOptions.IsPicEnabled = true;
    return false;
  }

  if (Option == "nopic") {
    getTargetStreamer().emitDirectiveOptionNoPIC();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    ParserOptions.IsPicEnabled = false;
    return false;
  }

  if (Option == "relax") {
    getTargetStreamer().emitDirectiveOptionRelax();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    setFeatureBits(SISC::FeatureRelax, "relax");
    return false;
  }

  if (Option == "norelax") {
    getTargetStreamer().emitDirectiveOptionNoRelax();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    clearFeatureBits(SISC::FeatureRelax, "relax");
    return false;
  }

  // Unknown option.
  Warning(Parser.getTok().getLoc(),
          "unknown option, expected 'push', 'pop', 'rvc', 'norvc', 'relax' or "
          "'norelax'");
  Parser.eatToEndOfStatement();
  return false;
}

/// parseDirectiveAttribute
///  ::= .attribute expression ',' ( expression | "string" )
///  ::= .attribute identifier ',' ( expression | "string" )
bool SISCAsmParser::parseDirectiveAttribute() {
  MCAsmParser &Parser = getParser();
  int64_t Tag;
  SMLoc TagLoc;
  TagLoc = Parser.getTok().getLoc();
  if (Parser.getTok().is(AsmToken::Identifier)) {
    StringRef Name = Parser.getTok().getIdentifier();
    Optional<unsigned> Ret =
        ELFAttrs::attrTypeFromString(Name, SISCAttrs::SISCAttributeTags);
    if (!Ret.hasValue()) {
      Error(TagLoc, "attribute name not recognised: " + Name);
      return false;
    }
    Tag = Ret.getValue();
    Parser.Lex();
  } else {
    const MCExpr *AttrExpr;

    TagLoc = Parser.getTok().getLoc();
    if (Parser.parseExpression(AttrExpr))
      return true;

    const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(AttrExpr);
    if (check(!CE, TagLoc, "expected numeric constant"))
      return true;

    Tag = CE->getValue();
  }

  if (Parser.parseToken(AsmToken::Comma, "comma expected"))
    return true;

  StringRef StringValue;
  int64_t IntegerValue = 0;
  bool IsIntegerValue = true;

  // RISC-V attributes have a string value if the tag number is odd
  // and an integer value if the tag number is even.
  if (Tag % 2)
    IsIntegerValue = false;

  SMLoc ValueExprLoc = Parser.getTok().getLoc();
  if (IsIntegerValue) {
    const MCExpr *ValueExpr;
    if (Parser.parseExpression(ValueExpr))
      return true;

    const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(ValueExpr);
    if (!CE)
      return Error(ValueExprLoc, "expected numeric constant");
    IntegerValue = CE->getValue();
  } else {
    if (Parser.getTok().isNot(AsmToken::String))
      return Error(Parser.getTok().getLoc(), "expected string constant");

    StringValue = Parser.getTok().getStringContents();
    Parser.Lex();
  }

  if (Parser.parseToken(AsmToken::EndOfStatement,
                        "unexpected token in '.attribute' directive"))
    return true;

  if (Tag == SISCAttrs::ARCH) {
    StringRef Arch = StringValue;
    if (Arch.consume_front("rv32"))
      clearFeatureBits(SISC::Feature64Bit, "64bit");
    else if (Arch.consume_front("rv64"))
      setFeatureBits(SISC::Feature64Bit, "64bit");
    else
      return Error(ValueExprLoc, "bad arch string " + Arch);

    while (!Arch.empty()) {
      if (Arch[0] == 'i')
        clearFeatureBits(SISC::FeatureRV32E, "e");
      else if (Arch[0] == 'e')
        setFeatureBits(SISC::FeatureRV32E, "e");
      else if (Arch[0] == 'g') {
        clearFeatureBits(SISC::FeatureRV32E, "e");
        setFeatureBits(SISC::FeatureStdExtM, "m");
        setFeatureBits(SISC::FeatureStdExtA, "a");
        setFeatureBits(SISC::FeatureStdExtF, "f");
        setFeatureBits(SISC::FeatureStdExtD, "d");
      } else if (Arch[0] == 'm')
        setFeatureBits(SISC::FeatureStdExtM, "m");
      else if (Arch[0] == 'a')
        setFeatureBits(SISC::FeatureStdExtA, "a");
      else if (Arch[0] == 'f')
        setFeatureBits(SISC::FeatureStdExtF, "f");
      else if (Arch[0] == 'd') {
        setFeatureBits(SISC::FeatureStdExtF, "f");
        setFeatureBits(SISC::FeatureStdExtD, "d");
      } else if (Arch[0] == 'c') {
        setFeatureBits(SISC::FeatureStdExtC, "c");
      } else
        return Error(ValueExprLoc, "bad arch string " + Arch);

      Arch = Arch.drop_front(1);
      int major = 0;
      int minor = 0;
      Arch.consumeInteger(10, major);
      Arch.consume_front("p");
      Arch.consumeInteger(10, minor);
      if (major != 0 || minor != 0) {
        Arch = Arch.drop_until([](char c) { return c == '_' || c == '"'; });
        Arch = Arch.drop_while([](char c) { return c == '_'; });
      }
    }
  }

  if (IsIntegerValue)
    getTargetStreamer().emitAttribute(Tag, IntegerValue);
  else {
    if (Tag != SISCAttrs::ARCH) {
      getTargetStreamer().emitTextAttribute(Tag, StringValue);
    } else {
      std::string formalArchStr = "rv32";
      if (getFeatureBits(SISC::Feature64Bit))
        formalArchStr = "rv64";
      if (getFeatureBits(SISC::FeatureRV32E))
        formalArchStr = (Twine(formalArchStr) + "e1p9").str();
      else
        formalArchStr = (Twine(formalArchStr) + "i2p0").str();

      if (getFeatureBits(SISC::FeatureStdExtM))
        formalArchStr = (Twine(formalArchStr) + "_m2p0").str();
      if (getFeatureBits(SISC::FeatureStdExtA))
        formalArchStr = (Twine(formalArchStr) + "_a2p0").str();
      if (getFeatureBits(SISC::FeatureStdExtF))
        formalArchStr = (Twine(formalArchStr) + "_f2p0").str();
      if (getFeatureBits(SISC::FeatureStdExtD))
        formalArchStr = (Twine(formalArchStr) + "_d2p0").str();
      if (getFeatureBits(SISC::FeatureStdExtC))
        formalArchStr = (Twine(formalArchStr) + "_c2p0").str();

      getTargetStreamer().emitTextAttribute(Tag, formalArchStr);
    }
  }

  return false;
}

void SISCAsmParser::emitToStreamer(MCStreamer &S, const MCInst &Inst) {
  MCInst CInst;
  bool Res = compressInst(CInst, Inst, getSTI(), S.getContext());
  if (Res)
    ++SISCNumInstrsCompressed;
  S.emitInstruction((Res ? CInst : Inst), getSTI());
}

void SISCAsmParser::emitLoadImm(Register DestReg, int64_t Value,
                                 MCStreamer &Out) {
  SISCMatInt::InstSeq Seq;
  SISCMatInt::generateInstSeq(Value, isRV64(), Seq);

  Register SrcReg = SISC::X0;
  for (SISCMatInt::Inst &Inst : Seq) {
    if (Inst.Opc == SISC::LUI) {
      emitToStreamer(
          Out, MCInstBuilder(SISC::LUI).addReg(DestReg).addImm(Inst.Imm));
    } else {
      emitToStreamer(
          Out, MCInstBuilder(Inst.Opc).addReg(DestReg).addReg(SrcReg).addImm(
                   Inst.Imm));
    }

    // Only the first instruction has X0 as its source.
    SrcReg = DestReg;
  }
}

void SISCAsmParser::emitAuipcInstPair(MCOperand DestReg, MCOperand TmpReg,
                                       const MCExpr *Symbol,
                                       SISCMCExpr::VariantKind VKHi,
                                       unsigned SecondOpcode, SMLoc IDLoc,
                                       MCStreamer &Out) {
  // A pair of instructions for PC-relative addressing; expands to
  //   TmpLabel: AUIPC TmpReg, VKHi(symbol)
  //             OP DestReg, TmpReg, %pcrel_lo(TmpLabel)
  MCContext &Ctx = getContext();

  MCSymbol *TmpLabel = Ctx.createTempSymbol(
      "pcrel_hi", /* AlwaysAddSuffix */ true, /* CanBeUnnamed */ false);
  Out.emitLabel(TmpLabel);

  const SISCMCExpr *SymbolHi = SISCMCExpr::create(Symbol, VKHi, Ctx);
  emitToStreamer(
      Out, MCInstBuilder(SISC::AUIPC).addOperand(TmpReg).addExpr(SymbolHi));

  const MCExpr *RefToLinkTmpLabel =
      SISCMCExpr::create(MCSymbolRefExpr::create(TmpLabel, Ctx),
                          SISCMCExpr::VK_SISC_PCREL_LO, Ctx);

  emitToStreamer(Out, MCInstBuilder(SecondOpcode)
                          .addOperand(DestReg)
                          .addOperand(TmpReg)
                          .addExpr(RefToLinkTmpLabel));
}

void SISCAsmParser::emitLoadLocalAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The load local address pseudo-instruction "lla" is used in PC-relative
  // addressing of local symbols:
  //   lla rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %pcrel_hi(symbol)
  //             ADDI rdest, rdest, %pcrel_lo(TmpLabel)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  emitAuipcInstPair(DestReg, DestReg, Symbol, SISCMCExpr::VK_SISC_PCREL_HI,
                    SISC::ADDI, IDLoc, Out);
}

void SISCAsmParser::emitLoadAddress(MCInst &Inst, SMLoc IDLoc,
                                     MCStreamer &Out) {
  // The load address pseudo-instruction "la" is used in PC-relative and
  // GOT-indirect addressing of global symbols:
  //   la rdest, symbol
  // expands to either (for non-PIC)
  //   TmpLabel: AUIPC rdest, %pcrel_hi(symbol)
  //             ADDI rdest, rdest, %pcrel_lo(TmpLabel)
  // or (for PIC)
  //   TmpLabel: AUIPC rdest, %got_pcrel_hi(symbol)
  //             Lx rdest, %pcrel_lo(TmpLabel)(rdest)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  unsigned SecondOpcode;
  SISCMCExpr::VariantKind VKHi;
  if (ParserOptions.IsPicEnabled) {
    SecondOpcode = isRV64() ? SISC::LD : SISC::LW;
    VKHi = SISCMCExpr::VK_SISC_GOT_HI;
  } else {
    SecondOpcode = SISC::ADDI;
    VKHi = SISCMCExpr::VK_SISC_PCREL_HI;
  }
  emitAuipcInstPair(DestReg, DestReg, Symbol, VKHi, SecondOpcode, IDLoc, Out);
}

void SISCAsmParser::emitLoadTLSIEAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The load TLS IE address pseudo-instruction "la.tls.ie" is used in
  // initial-exec TLS model addressing of global symbols:
  //   la.tls.ie rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %tls_ie_pcrel_hi(symbol)
  //             Lx rdest, %pcrel_lo(TmpLabel)(rdest)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  unsigned SecondOpcode = isRV64() ? SISC::LD : SISC::LW;
  emitAuipcInstPair(DestReg, DestReg, Symbol, SISCMCExpr::VK_SISC_TLS_GOT_HI,
                    SecondOpcode, IDLoc, Out);
}

void SISCAsmParser::emitLoadTLSGDAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The load TLS GD address pseudo-instruction "la.tls.gd" is used in
  // global-dynamic TLS model addressing of global symbols:
  //   la.tls.gd rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %tls_gd_pcrel_hi(symbol)
  //             ADDI rdest, rdest, %pcrel_lo(TmpLabel)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  emitAuipcInstPair(DestReg, DestReg, Symbol, SISCMCExpr::VK_SISC_TLS_GD_HI,
                    SISC::ADDI, IDLoc, Out);
}

void SISCAsmParser::emitLoadStoreSymbol(MCInst &Inst, unsigned Opcode,
                                         SMLoc IDLoc, MCStreamer &Out,
                                         bool HasTmpReg) {
  // The load/store pseudo-instruction does a pc-relative load with
  // a symbol.
  //
  // The expansion looks like this
  //
  //   TmpLabel: AUIPC tmp, %pcrel_hi(symbol)
  //             [S|L]X    rd, %pcrel_lo(TmpLabel)(tmp)
  MCOperand DestReg = Inst.getOperand(0);
  unsigned SymbolOpIdx = HasTmpReg ? 2 : 1;
  unsigned TmpRegOpIdx = HasTmpReg ? 1 : 0;
  MCOperand TmpReg = Inst.getOperand(TmpRegOpIdx);
  const MCExpr *Symbol = Inst.getOperand(SymbolOpIdx).getExpr();
  emitAuipcInstPair(DestReg, TmpReg, Symbol, SISCMCExpr::VK_SISC_PCREL_HI,
                    Opcode, IDLoc, Out);
}

bool SISCAsmParser::checkPseudoAddTPRel(MCInst &Inst,
                                         OperandVector &Operands) {
  assert(Inst.getOpcode() == SISC::PseudoAddTPRel && "Invalid instruction");
  assert(Inst.getOperand(2).isReg() && "Unexpected second operand kind");
  if (Inst.getOperand(2).getReg() != SISC::X4) {
    SMLoc ErrorLoc = ((SISCOperand &)*Operands[3]).getStartLoc();
    return Error(ErrorLoc, "the second input operand must be tp/x4 when using "
                           "%tprel_add modifier");
  }

  return false;
}

std::unique_ptr<SISCOperand> SISCAsmParser::defaultMaskRegOp() const {
  return SISCOperand::createReg(SISC::NoRegister, llvm::SMLoc(),
                                 llvm::SMLoc(), isRV64());
}

bool SISCAsmParser::validateInstruction(MCInst &Inst,
                                         OperandVector &Operands) {
  const MCInstrDesc &MCID = MII.get(Inst.getOpcode());
  unsigned TargetFlags =
      (MCID.TSFlags >> SISC::ConstraintOffset) & SISC::ConstraintMask;
  if (TargetFlags == SISC::NoConstraint)
    return false;

  unsigned DestReg = Inst.getOperand(0).getReg();
  // Operands[1] will be the first operand, DestReg.
  SMLoc Loc = Operands[1]->getStartLoc();
  if ((TargetFlags == SISC::WidenV) || (TargetFlags == SISC::WidenW) ||
      (TargetFlags == SISC::SlideUp) || (TargetFlags == SISC::Vrgather) ||
      (TargetFlags == SISC::Vcompress)) {
    if (TargetFlags != SISC::WidenW) {
      unsigned Src2Reg = Inst.getOperand(1).getReg();
      if (DestReg == Src2Reg)
        return Error(Loc, "The destination vector register group cannot overlap"
                          " the source vector register group.");
      if (TargetFlags == SISC::WidenV) {
        // Assume DestReg LMUL is 2 at least for widening/narrowing operations.
        if (DestReg + 1 == Src2Reg)
          return Error(Loc,
                       "The destination vector register group cannot overlap"
                       " the source vector register group.");
      }
    }
    if (Inst.getOperand(2).isReg()) {
      unsigned Src1Reg = Inst.getOperand(2).getReg();
      if (DestReg == Src1Reg)
        return Error(Loc, "The destination vector register group cannot overlap"
                          " the source vector register group.");
      if (TargetFlags == SISC::WidenV || TargetFlags == SISC::WidenW) {
        // Assume DestReg LMUL is 2 at least for widening/narrowing operations.
        if (DestReg + 1 == Src1Reg)
          return Error(Loc,
                       "The destination vector register group cannot overlap"
                       " the source vector register group.");
      }
    }
    if (Inst.getNumOperands() == 4) {
      unsigned MaskReg = Inst.getOperand(3).getReg();

      if (DestReg == MaskReg)
        return Error(Loc, "The destination vector register group cannot overlap"
                          " the mask register.");
    }
  } else if (TargetFlags == SISC::Narrow) {
    unsigned Src2Reg = Inst.getOperand(1).getReg();
    if (DestReg == Src2Reg)
      return Error(Loc, "The destination vector register group cannot overlap"
                        " the source vector register group.");
    // Assume Src2Reg LMUL is 2 at least for widening/narrowing operations.
    if (DestReg == Src2Reg + 1)
      return Error(Loc, "The destination vector register group cannot overlap"
                        " the source vector register group.");
  } else if (TargetFlags == SISC::WidenCvt || TargetFlags == SISC::Iota) {
    unsigned Src2Reg = Inst.getOperand(1).getReg();
    if (DestReg == Src2Reg)
      return Error(Loc, "The destination vector register group cannot overlap"
                        " the source vector register group.");
    if (TargetFlags == SISC::WidenCvt) {
      // Assume DestReg LMUL is 2 at least for widening/narrowing operations.
      if (DestReg + 1 == Src2Reg)
        return Error(Loc, "The destination vector register group cannot overlap"
                          " the source vector register group.");
    }
    if (Inst.getNumOperands() == 3) {
      unsigned MaskReg = Inst.getOperand(2).getReg();

      if (DestReg == MaskReg)
        return Error(Loc, "The destination vector register group cannot overlap"
                          " the mask register.");
    }
  }
  return false;
}

bool SISCAsmParser::processInstruction(MCInst &Inst, SMLoc IDLoc,
                                        OperandVector &Operands,
                                        MCStreamer &Out) {
  Inst.setLoc(IDLoc);

  switch (Inst.getOpcode()) {
  default:
    break;
  case SISC::PseudoLI: {
    Register Reg = Inst.getOperand(0).getReg();
    const MCOperand &Op1 = Inst.getOperand(1);
    if (Op1.isExpr()) {
      // We must have li reg, %lo(sym) or li reg, %pcrel_lo(sym) or similar.
      // Just convert to an addi. This allows compatibility with gas.
      emitToStreamer(Out, MCInstBuilder(SISC::ADDI)
                              .addReg(Reg)
                              .addReg(SISC::X0)
                              .addExpr(Op1.getExpr()));
      return false;
    }
    int64_t Imm = Inst.getOperand(1).getImm();
    // On RV32 the immediate here can either be a signed or an unsigned
    // 32-bit number. Sign extension has to be performed to ensure that Imm
    // represents the expected signed 64-bit number.
    if (!isRV64())
      Imm = SignExtend64<32>(Imm);
    emitLoadImm(Reg, Imm, Out);
    return false;
  }
  case SISC::PseudoLLA:
    emitLoadLocalAddress(Inst, IDLoc, Out);
    return false;
  case SISC::PseudoLA:
    emitLoadAddress(Inst, IDLoc, Out);
    return false;
  case SISC::PseudoLA_TLS_IE:
    emitLoadTLSIEAddress(Inst, IDLoc, Out);
    return false;
  case SISC::PseudoLA_TLS_GD:
    emitLoadTLSGDAddress(Inst, IDLoc, Out);
    return false;
  case SISC::PseudoLB:
    emitLoadStoreSymbol(Inst, SISC::LB, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case SISC::PseudoLBU:
    emitLoadStoreSymbol(Inst, SISC::LBU, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case SISC::PseudoLH:
    emitLoadStoreSymbol(Inst, SISC::LH, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case SISC::PseudoLHU:
    emitLoadStoreSymbol(Inst, SISC::LHU, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case SISC::PseudoLW:
    emitLoadStoreSymbol(Inst, SISC::LW, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case SISC::PseudoLWU:
    emitLoadStoreSymbol(Inst, SISC::LWU, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case SISC::PseudoLD:
    emitLoadStoreSymbol(Inst, SISC::LD, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case SISC::PseudoFLW:
    emitLoadStoreSymbol(Inst, SISC::FLW, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case SISC::PseudoFLD:
    emitLoadStoreSymbol(Inst, SISC::FLD, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case SISC::PseudoSB:
    emitLoadStoreSymbol(Inst, SISC::SB, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case SISC::PseudoSH:
    emitLoadStoreSymbol(Inst, SISC::SH, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case SISC::PseudoSW:
    emitLoadStoreSymbol(Inst, SISC::SW, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case SISC::PseudoSD:
    emitLoadStoreSymbol(Inst, SISC::SD, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case SISC::PseudoFSW:
    emitLoadStoreSymbol(Inst, SISC::FSW, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case SISC::PseudoFSD:
    emitLoadStoreSymbol(Inst, SISC::FSD, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case SISC::PseudoAddTPRel:
    if (checkPseudoAddTPRel(Inst, Operands))
      return true;
    break;
  }

  emitToStreamer(Out, Inst);
  return false;
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeSISCAsmParser() {
  RegisterMCAsmParser<SISCAsmParser> X(getTheSISC32Target());
  RegisterMCAsmParser<SISCAsmParser> Y(getTheSISC64Target());
}

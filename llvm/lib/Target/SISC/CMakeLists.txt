set(LLVM_TARGET_DEFINITIONS SISC.td)

tablegen(LLVM SISCGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM SISCGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM SISCGenCompressInstEmitter.inc -gen-compress-inst-emitter)
tablegen(LLVM SISCGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM SISCGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM SISCGenGlobalISel.inc -gen-global-isel)
tablegen(LLVM SISCGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM SISCGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM SISCGenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM SISCGenRegisterBank.inc -gen-register-bank)
tablegen(LLVM SISCGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM SISCGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM SISCGenSystemOperands.inc -gen-searchable-tables)

add_public_tablegen_target(SISCCommonTableGen)

add_llvm_target(SISCCodeGen
  SISCAsmPrinter.cpp
  SISCCallLowering.cpp
  SISCExpandAtomicPseudoInsts.cpp
  SISCExpandPseudoInsts.cpp
  SISCFrameLowering.cpp
  SISCInstrInfo.cpp
  SISCInstructionSelector.cpp
  SISCISelDAGToDAG.cpp
  SISCISelLowering.cpp
  SISCLegalizerInfo.cpp
  SISCMCInstLower.cpp
  SISCMergeBaseOffset.cpp
  SISCRegisterBankInfo.cpp
  SISCRegisterInfo.cpp
  SISCSubtarget.cpp
  SISCTargetMachine.cpp
  SISCTargetObjectFile.cpp
  SISCTargetTransformInfo.cpp
  )

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
add_subdirectory(Utils)

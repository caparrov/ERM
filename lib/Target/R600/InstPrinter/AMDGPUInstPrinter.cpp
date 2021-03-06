//===-- AMDGPUInstPrinter.cpp - AMDGPU MC Inst -> ASM ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
// \file
//===----------------------------------------------------------------------===//

#include "AMDGPUInstPrinter.h"
#include "MCTargetDesc/AMDGPUMCTargetDesc.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"

using namespace llvm;

void AMDGPUInstPrinter::printInst(const MCInst *MI, raw_ostream &OS,
                             StringRef Annot) {
  OS.flush();
  printInstruction(MI, OS);

  printAnnotation(OS, Annot);
}

void AMDGPUInstPrinter::printRegOperand(unsigned reg, raw_ostream &O) {
  switch (reg) {
  case AMDGPU::VCC:
    O << "vcc";
    return;
  case AMDGPU::SCC:
    O << "scc";
    return;
  case AMDGPU::EXEC:
    O << "exec";
    return;
  case AMDGPU::M0:
    O << "m0";
    return;
  default:
    break;
  }

  // It's seems there's no way to use SIRegisterInfo here, and dealing with the
  // giant enum of all the different shifted sets of registers is pretty
  // unmanagable, so parse the name and reformat it to be prettier.
  StringRef Name(getRegisterName(reg));

  std::pair<StringRef, StringRef> Split = Name.split('_');
  StringRef SubRegName = Split.first;
  StringRef Rest = Split.second;

  if (SubRegName.size() <= 4) { // Must at least be as long as "SGPR"/"VGPR".
    O << Name;
    return;
  }

  unsigned RegIndex;
  StringRef RegIndexStr = SubRegName.drop_front(4);

  if (RegIndexStr.getAsInteger(10, RegIndex)) {
    O << Name;
    return;
  }

  if (SubRegName.front() == 'V')
    O << 'v';
  else if (SubRegName.front() == 'S')
    O << 's';
  else {
    O << Name;
    return;
  }

  if (Rest.empty()) // Only 1 32-bit register
    O << RegIndex;
  else {
    unsigned NumReg = Rest.count('_') + 2;
    O << '[' << RegIndex << ':' << (RegIndex + NumReg - 1) << ']';
  }
}

void AMDGPUInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                     raw_ostream &O) {

  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isReg()) {
    switch (Op.getReg()) {
    // This is the default predicate state, so we don't need to print it.
    case AMDGPU::PRED_SEL_OFF:
      break;

    default:
      printRegOperand(Op.getReg(), O);
      break;
    }
  } else if (Op.isImm()) {
    O << Op.getImm();
  } else if (Op.isFPImm()) {
    O << Op.getFPImm();
  } else if (Op.isExpr()) {
    const MCExpr *Exp = Op.getExpr();
    Exp->print(O);
  } else {
    assert(!"unknown operand type in printOperand");
  }
}

void AMDGPUInstPrinter::printInterpSlot(const MCInst *MI, unsigned OpNum,
                                        raw_ostream &O) {
  unsigned Imm = MI->getOperand(OpNum).getImm();

  if (Imm == 2) {
    O << "P0";
  } else if (Imm == 1) {
    O << "P20";
  } else if (Imm == 0) {
    O << "P10";
  } else {
    assert(!"Invalid interpolation parameter slot");
  }
}

void AMDGPUInstPrinter::printMemOperand(const MCInst *MI, unsigned OpNo,
                                        raw_ostream &O) {
  printOperand(MI, OpNo, O);
  O  << ", ";
  printOperand(MI, OpNo + 1, O);
}

void AMDGPUInstPrinter::printIfSet(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O, StringRef Asm,
                                   StringRef Default) {
  const MCOperand &Op = MI->getOperand(OpNo);
  assert(Op.isImm());
  if (Op.getImm() == 1) {
    O << Asm;
  } else {
    O << Default;
  }
}

void AMDGPUInstPrinter::printAbs(const MCInst *MI, unsigned OpNo,
                                 raw_ostream &O) {
  printIfSet(MI, OpNo, O, "|");
}

void AMDGPUInstPrinter::printClamp(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O) {
  printIfSet(MI, OpNo, O, "_SAT");
}

void AMDGPUInstPrinter::printLiteral(const MCInst *MI, unsigned OpNo,
                                     raw_ostream &O) {
  union Literal {
    float f;
    int32_t i;
  } L;

  L.i = MI->getOperand(OpNo).getImm();
  O << L.i << "(" << L.f << ")";
}

void AMDGPUInstPrinter::printLast(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  printIfSet(MI, OpNo, O.indent(25 - O.GetNumBytesInBuffer()), "*", " ");
}

void AMDGPUInstPrinter::printNeg(const MCInst *MI, unsigned OpNo,
                                 raw_ostream &O) {
  printIfSet(MI, OpNo, O, "-");
}

void AMDGPUInstPrinter::printOMOD(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  switch (MI->getOperand(OpNo).getImm()) {
  default: break;
  case 1:
    O << " * 2.0";
    break;
  case 2:
    O << " * 4.0";
    break;
  case 3:
    O << " / 2.0";
    break;
  }
}

void AMDGPUInstPrinter::printRel(const MCInst *MI, unsigned OpNo,
                                 raw_ostream &O) {
  printIfSet(MI, OpNo, O, "+");
}

void AMDGPUInstPrinter::printUpdateExecMask(const MCInst *MI, unsigned OpNo,
                                            raw_ostream &O) {
  printIfSet(MI, OpNo, O, "ExecMask,");
}

void AMDGPUInstPrinter::printUpdatePred(const MCInst *MI, unsigned OpNo,
                                        raw_ostream &O) {
  printIfSet(MI, OpNo, O, "Pred,");
}

void AMDGPUInstPrinter::printWrite(const MCInst *MI, unsigned OpNo,
                                       raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.getImm() == 0) {
    O << " (MASKED)";
  }
}

void AMDGPUInstPrinter::printSel(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  const char * chans = "XYZW";
  int sel = MI->getOperand(OpNo).getImm();

  int chan = sel & 3;
  sel >>= 2;

  if (sel >= 512) {
    sel -= 512;
    int cb = sel >> 12;
    sel &= 4095;
    O << cb << "[" << sel << "]";
  } else if (sel >= 448) {
    sel -= 448;
    O << sel;
  } else if (sel >= 0){
    O << sel;
  }

  if (sel >= 0)
    O << "." << chans[chan];
}

void AMDGPUInstPrinter::printBankSwizzle(const MCInst *MI, unsigned OpNo,
                                         raw_ostream &O) {
  int BankSwizzle = MI->getOperand(OpNo).getImm();
  switch (BankSwizzle) {
  case 1:
    O << "BS:VEC_021/SCL_122";
    break;
  case 2:
    O << "BS:VEC_120/SCL_212";
    break;
  case 3:
    O << "BS:VEC_102/SCL_221";
    break;
  case 4:
    O << "BS:VEC_201";
    break;
  case 5:
    O << "BS:VEC_210";
    break;
  default:
    break;
  }
  return;
}

void AMDGPUInstPrinter::printRSel(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  unsigned Sel = MI->getOperand(OpNo).getImm();
  switch (Sel) {
  case 0:
    O << "X";
    break;
  case 1:
    O << "Y";
    break;
  case 2:
    O << "Z";
    break;
  case 3:
    O << "W";
    break;
  case 4:
    O << "0";
    break;
  case 5:
    O << "1";
    break;
  case 7:
    O << "_";
    break;
  default:
    break;
  }
}

void AMDGPUInstPrinter::printCT(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  unsigned CT = MI->getOperand(OpNo).getImm();
  switch (CT) {
  case 0:
    O << "U";
    break;
  case 1:
    O << "N";
    break;
  default:
    break;
  }
}

void AMDGPUInstPrinter::printKCache(const MCInst *MI, unsigned OpNo,
                                    raw_ostream &O) {
  int KCacheMode = MI->getOperand(OpNo).getImm();
  if (KCacheMode > 0) {
    int KCacheBank = MI->getOperand(OpNo - 2).getImm();
    O << "CB" << KCacheBank <<":";
    int KCacheAddr = MI->getOperand(OpNo + 2).getImm();
    int LineSize = (KCacheMode == 1)?16:32;
    O << KCacheAddr * 16 << "-" << KCacheAddr * 16 + LineSize;
  }
}

void AMDGPUInstPrinter::printWaitFlag(const MCInst *MI, unsigned OpNo,
                                      raw_ostream &O) {
  // Note: Mask values are taken from SIInsertWaits.cpp and not from ISA docs
  // SIInsertWaits.cpp bits usage does not match ISA docs description but it
  // works so it might be a misprint in docs.
  unsigned SImm16 = MI->getOperand(OpNo).getImm();
  unsigned Vmcnt = SImm16 & 0xF;
  unsigned Expcnt = (SImm16 >> 4) & 0xF;
  unsigned Lgkmcnt = (SImm16 >> 8) & 0xF;
  if (Vmcnt != 0xF)
    O << "vmcnt(" << Vmcnt << ") ";
  if (Expcnt != 0x7)
    O << "expcnt(" << Expcnt << ") ";
  if (Lgkmcnt != 0x7)
    O << "lgkmcnt(" << Lgkmcnt << ")";
}

#include "AMDGPUGenAsmWriter.inc"

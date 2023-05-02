//===-- src/cpu.h - CPU class definition -------*- C++ -*-===//
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the declaration of the CPU class, which is the base class
/// containing the entire architecture of the emulated 8085 microprocessor
///
//===----------------------------------------------------------------------===//

#ifndef CPU_H
#define CPU_H

#include <array>
#include <fstream>

namespace emu {

using Byte = unsigned char;
using Word = unsigned short;

struct Memory {
  static constexpr unsigned MEM_LIMIT = 1024 * 64;
  std::array<Byte, MEM_LIMIT> data;

  void initialise();
  bool initialise(std::ifstream &input_file);

  // Read a byte
  Byte operator[](unsigned p_address) const { return data[p_address]; }

  // Write a byte
  Byte &operator[](unsigned p_address) { return data[p_address]; }
};

struct StatusFlags {
  bool c; // carry flag
  bool p; // parity flag
  bool a; // auxilary carry flag
  bool z; // zero flag
  bool s; // sign flag
};

class CPU {
public:
  CPU() {}
  CPU(const CPU &other);
  CPU(CPU &&other) noexcept = delete;
  virtual ~CPU() noexcept {}

  void reset(Memory &mem);
  void execute(int &cycles, Memory &mem);

  Word readRegisterPair(const Byte &reg1, const Byte &reg2);
  void writeRegisterPair(Byte &reg1, Byte &reg2, Word &data);
  Byte fetchByte(int &cycles, const Memory &mem);
  Word fetchWord(int &cycles, const Memory &mem);

  /** opcodes */

  static constexpr Byte
      /*  Data Transfer group */

      // Move
      MOV_AA = 0x7F,
      MOV_AB = 0x78, MOV_AC = 0x79, MOV_AD = 0x7A, MOV_AE = 0x7B, MOV_AH = 0x7C,
      MOV_AL = 0x7D, MOV_AM = 0x7E,

      MOV_BA = 0x47, MOV_BB = 0x40, MOV_BC = 0x41, MOV_BD = 0x42, MOV_BE = 0x43,
      MOV_BH = 0x44, MOV_BL = 0x45, MOV_BM = 0x46,

      MOV_CA = 0x4F, MOV_CB = 0x48, MOV_CC = 0x49, MOV_CD = 0x4A, MOV_CE = 0x4B,
      MOV_CH = 0x4C, MOV_CL = 0x4D, MOV_CM = 0x4E,

      MOV_DA = 0x57, MOV_DB = 0x50, MOV_DC = 0x51, MOV_DD = 0x52, MOV_DE = 0x53,
      MOV_DH = 0x54, MOV_DL = 0x55, MOV_DM = 0x56,

      MOV_EA = 0x5F, MOV_EB = 0x58, MOV_EC = 0x59, MOV_ED = 0x5A, MOV_EE = 0x5B,
      MOV_EH = 0x5C, MOV_EL = 0x5D, MOV_EM = 0x5E,

      MOV_HA = 0x67, MOV_HB = 0x60, MOV_HC = 0x61, MOV_HD = 0x62, MOV_HE = 0x63,
      MOV_HH = 0x64, MOV_HL = 0x65, MOV_HM = 0x66,

      MOV_LA = 0x6F, MOV_LB = 0x68, MOV_LC = 0x69, MOV_LD = 0x6A, MOV_LE = 0x6B,
      MOV_LH = 0x6C, MOV_LL = 0x6D, MOV_LM = 0x6E,

      MOV_MA = 0x77, MOV_MB = 0x70, MOV_MC = 0x71, MOV_MD = 0x72, MOV_ME = 0x73,
      MOV_MH = 0x74, MOV_ML = 0x75,

      // Move immediate
      MVI_A = 0x3E, MVI_B = 0x06, MVI_C = 0x0E, MVI_D = 0x16, MVI_E = 0x1E,
      MVI_H = 0x26, MVI_L = 0x2E, MVI_M = 0x36,

      // load immediate (reg pair)
      LXI_B = 0x01, LXI_D = 0x11, LXI_H = 0x21, LXI_SP = 0x31,

      // load/store A direct ( from register pair )
      LDAX_B = 0x0A, LDAX_D = 0x1A, STAX_B = 0x02, STAX_D = 0x12,

      // load/store A direct ( from memory )
      LDA = 0x3A, STA = 0x32,

      // load/store HL direct
      LHLD = 0x2A, SHLD = 0x22,

      // exchange HL and DE
      XCHNG = 0xEB,

      /** Branch group*/

      // Jump
      JMP = 0xC3, JNZ = 0xC2, JZ = 0xCA, JNC = 0xD2, JC = 0xDA, JPO = 0xE2,
      JPE = 0xEA, JP = 0xF2, JM = 0xFA,

      // Call
      CALL = 0xCD, CNZ = 0xC4, CZ = 0xCC, CNC = 0xD4, CC = 0xDC, CPO = 0xE6,
      CPE = 0xEC, CP = 0xF4, CM = 0xFC,

      // Return
      RET = 0xC9, RNZ = 0xC0, RZ = 0xC8, RNC = 0xD0, RC = 0xD8, RPO = 0xE0,
      RPE = 0xE8, RP = 0xF0, RM = 0xF8,

      // jump indirect
      PCHL = 0xE9;

  /** debug */

  /** getters */
  auto getProgramCounter() const { return programCounter; }
  auto getStackPointer() const { return stackPointer; }
  auto getStatusFlags() const { return statusFlags; }

  auto getARegister() const { return A; }
  auto getBRegister() const { return B; }
  auto getCRegister() const { return C; }
  auto getDRegister() const { return D; }
  auto getERegister() const { return E; }
  auto getHRegister() const { return H; }
  auto getLRegister() const { return L; }

private:
  StatusFlags statusFlags; // program status word
  Word programCounter;     // program counter
  Word stackPointer;       // stack pointer

  Byte A, B, C, D, E, H, L;

  /** instruction set */
  void loadAccumulator(int &cycles, const Memory &mem);
  void storeAccumulator(int &cycles, Memory &mem);

  void moveRegister(Byte &destination, Byte &source);
  void moveIndirectTo(Byte &destination, const Memory &mem, int &cycles);
  void moveIndirectFrom(const Byte &source, Memory &mem, int &cycles);
  void moveImmediate(Byte &destination, int &cycles, const Memory &mem);
  void loadImmediate(Byte &reg1, Byte &reg2, int &cycles, const Memory &mem);
  void loadImmediate(Word &reg, int &cycles, const Memory &mem);

  void unconditionalJump(int &cycles, const Memory &mem);

  // conditional jump
  void jumpIfNotZero(int &cycles, const Memory &mem);
  void jumpIfZero(int &cycles, const Memory &mem);
  void jumpIfNotCarry(int &cycles, const Memory &mem);
  void jumpIfCarry(int &cycles, const Memory &mem);
  void jumpIfOddParity(int &cycles, const Memory &mem);
  void jumpIfEvenParity(int &cycles, const Memory &mem);
  void jumpIfPositive(int &cycles, const Memory &mem);
  void jumpIfMinus(int &cycles, const Memory &mem);

  // call
  void unconditionalCall(int &cycles, Memory &mem);

  // conditional call
  void callIfNotZero(int &cycles, Memory &mem);
  void callIfZero(int &cycles, Memory &mem);
  void callIfNotCarry(int &cycles, Memory &mem);
  void callIfCarry(int &cycles, Memory &mem);
  void callIfOddParity(int &cycles, Memory &mem);
  void callIfEvenParity(int &cycles, Memory &mem);
  void callIfPositive(int &cycles, Memory &mem);
  void callIfMinus(int &cycles, Memory &mem);

  // stack
  void stackPush(int &cycles, Memory &mem, const Byte &data);
  Byte stackPop(int &cycles, const Memory &mem);
};

} // namespace emu

#endif /* CPU_H */

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

namespace emu {

using Byte = unsigned char;
using Word = unsigned short;

struct Memory {
  static constexpr unsigned MEM_LIMIT = 1024 * 64;
  std::array<Byte, MEM_LIMIT> data;

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
  CPU();
  CPU(const CPU &other);
  CPU(CPU &&other) noexcept = delete;
  virtual ~CPU() noexcept;

  // Instruction set

  // Debug

private:
  StatusFlags flagRegister;
  Word pc; // program counter
  Word sp; // stack pointer

  Byte A, B, C, D, E, H, L;
  Memory memory;
};

} // namespace emu

#endif /* CPU_H */

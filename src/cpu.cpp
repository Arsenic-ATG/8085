#include "cpu.h"
#include <iostream>

using namespace emu;

//  memeory


/**
 * Zero initilise the memory of the processor.
 */
void Memory::initialise() { data = {}; }

/**
 * Intialise memory with the program provided in the input file, the
 * function also does bound checking and doesn't write to memory
 * incase the data to be written to the memory is more than the memory
 * iteself.
 */
bool Memory::initialise(std::ifstream &input_file) {
  if (!input_file.is_open()) {
    std::cerr << "can't open the file\n";
    return false;
  }

  if (input_file.gcount() >= static_cast<long>(data.size())) {
    std::cerr << "program is bigger than the memory\n";
    return false;
  }

  input_file.read(reinterpret_cast<char *>(data.data()), data.size());
  return true;
}
// class cpu

void CPU::reset(Memory &mem) {
  programCounter = A = B = C = D = E = H = L = 0;
  stackPointer = 0xFFFF;
  statusFlags = {};
  mem.initialise();
}

void CPU::execute(int &cycles, Memory &mem) {
  while (cycles > 0) {
    auto lastInstructionLocation = programCounter;
    auto instruction = fetchByte(cycles, mem);
    switch (instruction) {

    case MOV_AA:
    case MOV_BB:
    case MOV_CC:
    case MOV_DD:
    case MOV_EE:
    case MOV_HH:
    case MOV_LL:
      break;

    case MOV_AB: {
      moveRegister(A, B);
    } break;
    case MOV_AC: {
      moveRegister(A, C);
    } break;
    case MOV_AD: {
      moveRegister(A, D);
    } break;
    case MOV_AE: {
      moveRegister(A, E);
    } break;
    case MOV_AH: {
      moveRegister(A, H);
    } break;
    case MOV_AL: {
      moveRegister(A, L);
    } break;

    case MOV_BA: {
      moveRegister(B, A);
    } break;
    case MOV_BC: {
      moveRegister(B, C);
    } break;
    case MOV_BD: {
      moveRegister(B, D);
    } break;
    case MOV_BE: {
      moveRegister(B, E);
    } break;
    case MOV_BH: {
      moveRegister(B, H);
    } break;
    case MOV_BL: {
      moveRegister(B, L);
    } break;

    case MOV_CA: {
      moveRegister(C, A);
    } break;
    case MOV_CB: {
      moveRegister(C, B);
    } break;
    case MOV_CD: {
      moveRegister(C, D);
    } break;
    case MOV_CE: {
      moveRegister(C, E);
    } break;
    case MOV_CH: {
      moveRegister(C, H);
    } break;
    case MOV_CL: {
      moveRegister(C, L);
    } break;

    case MOV_DA: {
      moveRegister(D, A);
    } break;
    case MOV_DB: {
      moveRegister(D, B);
    } break;
    case MOV_DC: {
      moveRegister(D, C);
    } break;
    case MOV_DE: {
      moveRegister(D, E);
    } break;
    case MOV_DH: {
      moveRegister(D, H);
    } break;
    case MOV_DL: {
      moveRegister(D, L);
    } break;

    case MOV_EA: {
      moveRegister(E, A);
    } break;
    case MOV_EB: {
      moveRegister(E, B);
    } break;
    case MOV_EC: {
      moveRegister(E, C);
    } break;
    case MOV_ED: {
      moveRegister(E, E);
    } break;
    case MOV_EH: {
      moveRegister(E, H);
    } break;
    case MOV_EL: {
      moveRegister(E, L);
    } break;

    case MOV_HA: {
      moveRegister(H, A);
    } break;
    case MOV_HB: {
      moveRegister(H, B);
    } break;
    case MOV_HC: {
      moveRegister(H, C);
    } break;
    case MOV_HD: {
      moveRegister(H, E);
    } break;
    case MOV_HE: {
      moveRegister(H, E);
    } break;
    case MOV_HL: {
      moveRegister(H, L);
    } break;

    case MOV_LA: {
      moveRegister(L, A);
    } break;
    case MOV_LB: {
      moveRegister(L, B);
    } break;
    case MOV_LC: {
      moveRegister(L, C);
    } break;
    case MOV_LD: {
      moveRegister(L, E);
    } break;
    case MOV_LH: {
      moveRegister(L, H);
    } break;
    case MOV_LE: {
      moveRegister(L, E);
    } break;

    // register indirect addressing mode
    case MOV_AM: {
      moveIndirectTo(A, mem, cycles);
    } break;
    case MOV_BM: {
      moveIndirectTo(B, mem, cycles);
    } break;
    case MOV_CM: {
      moveIndirectTo(A, mem, cycles);
    } break;
    case MOV_DM: {
      moveIndirectTo(A, mem, cycles);
    } break;
    case MOV_EM: {
      moveIndirectTo(E, mem, cycles);
    } break;
    case MOV_HM: {
      moveIndirectTo(H, mem, cycles);
    } break;
    case MOV_LM: {
      moveIndirectTo(L, mem, cycles);
    } break;

    case MOV_MA: {
      moveIndirectFrom(A, mem, cycles);
    } break;
    case MOV_MB: {
      moveIndirectFrom(B, mem, cycles);
    } break;
    case MOV_MC: {
      moveIndirectFrom(A, mem, cycles);
    } break;
    case MOV_MD: {
      moveIndirectFrom(A, mem, cycles);
    } break;
    case MOV_ME: {
      moveIndirectFrom(E, mem, cycles);
    } break;
    case MOV_MH: {
      moveIndirectFrom(H, mem, cycles);
    } break;
    case MOV_ML: {
      moveIndirectFrom(L, mem, cycles);
    } break;

    // move immediate
    case MVI_A: {
      moveImmediate(A, cycles, mem);
    } break;
    case MVI_B: {
      moveImmediate(B, cycles, mem);
    } break;
    case MVI_C: {
      moveImmediate(C, cycles, mem);
    } break;
    case MVI_D: {
      moveImmediate(D, cycles, mem);
    } break;
    case MVI_E: {
      moveImmediate(E, cycles, mem);
    } break;
    case MVI_H: {
      moveImmediate(H, cycles, mem);
    } break;
    case MVI_L: {
      moveImmediate(L, cycles, mem);
    } break;
    case MVI_M: {
      // memory address to where the data should be tranfered would be
      // present in H-L register pair
      auto address = readRegisterPair(H, L);
      cycles--;
      auto destination = mem[address];
      moveImmediate(destination, cycles, mem);
    } break;

    // load/store immediate (register pair)
    case LXI_B: {
      loadImmediate(B, C, cycles, mem);
    } break;
    case LXI_D: {
      loadImmediate(D, E, cycles, mem);
    } break;
    case LXI_H: {
      loadImmediate(H, L, cycles, mem);
    } break;
    case LXI_SP: {
      loadImmediate(stackPointer, cycles, mem);
    } break;

    // load/store accumulator ( from register pair )
    case LDAX_B: {
      auto address = readRegisterPair(B, C);
      cycles--;
      A = mem[address];
    } break;
    case LDAX_D: {
      auto address = readRegisterPair(D, E);
      cycles--;
      A = mem[address];
    } break;
    case STAX_B: {
      auto address = readRegisterPair(B, C);
      cycles--;
      mem[address] = A;
    } break;
    case STAX_D: {
      auto address = readRegisterPair(D, E);
      cycles--;
      mem[address] = A;
    } break;

    // load store accumulator directly
    case LDA: {
      loadAccumulator(cycles, mem);
    } break;
    case STA: {
      storeAccumulator(cycles, mem);
    }
    // load/store HL direct
    case LHLD: {
      auto address = fetchWord(cycles, mem);
      H = mem[address];
      L = mem[address + 1];
    } break;
    case SHLD: {
      auto address = fetchWord(cycles, mem);
      mem[address] = L;
      mem[address + 1] = H;
    } break;

    // exchange HL and DE
    case XCHNG: {
      auto dataHL = readRegisterPair(H, L);
      auto dataDE = readRegisterPair(D, E);
      writeRegisterPair(H, L, dataDE);
      writeRegisterPair(D, E, dataHL);
    } break;

    // jump
    case JMP: {
      unconditionalJump(cycles, mem);
    } break;

    // contitional jumps
    case JNZ: {
      jumpIfNotZero(cycles, mem);
    } break;
    case JZ: {
      jumpIfZero(cycles, mem);
    } break;
    case JNC: {
      jumpIfNotCarry(cycles, mem);
    } break;
    case JC: {
      jumpIfCarry(cycles, mem);
    } break;
    case JPO: {
      jumpIfOddParity(cycles, mem);
    } break;
    case JP: {
      jumpIfEvenParity(cycles, mem);
    } break;
    case JPE: {
      jumpIfPositive(cycles, mem);
    } break;
    case JM: {
      jumpIfMinus(cycles, mem);
    } break;

    // call instructions
    case CALL: {
      unconditionalCall(cycles, mem);
    } break;

    // conditional call
    case CNZ: {
      callIfNotZero(cycles, mem);
    } break;
    case CZ: {
      callIfZero(cycles, mem);
    } break;
    case CNC: {
      callIfNotCarry(cycles, mem);
    } break;
    case CC: {
      callIfCarry(cycles, mem);
    } break;
    case CPO: {
      callIfOddParity(cycles, mem);
    } break;
    case CP: {
      callIfEvenParity(cycles, mem);
    } break;
    case CPE: {
      callIfPositive(cycles, mem);
    } break;
    case CM: {
      callIfMinus(cycles, mem);
    } break;

    default:
      // throw for unknown instruction.
      std::cerr << "unknown instruction enountered "
                << static_cast<int>(instruction)
                << " at memory location : " << lastInstructionLocation << "\n";
    }
  }
}

/**
 * Read the 16-bit (2-Bytes/1-Word) data from register pair reg1-reg2.
 */
Word CPU::readRegisterPair(const Byte &reg1, const Byte &reg2) {
  // TODO: assert that reg1 and reg2 are valid register pair.
  Word data = reg1 << 8;
  data |= reg2;
  return data;
}

/**
 * Write 16-bit (2-Bytes/1-Word) data to register pair reg1-reg2.
 */
void CPU::writeRegisterPair(Byte &reg1, Byte &reg2, Word &data) {
  // TODO: assert that reg1 and reg2 are valid register pair.
  reg1 = data >> 8;
  reg2 = static_cast<Byte>(data);
}

/**
 * Fetch one byte data pointed by the Progam counter from memory MEM.
 */
Byte CPU::fetchByte(int &cycles, const Memory &mem) {
  auto currentData = mem[programCounter];
  --cycles;
  ++programCounter;
  return currentData;
}

/**
 * Fetch one word ( 2-bytes ) data pointed by the Progam counter from
 * memory MEM. Assuming the platform is little-endien, we would first
 * find the lower byte of address then lower byte.
 */
Word CPU::fetchWord(int &cycles, const Memory &mem) {
  Word currentData = mem[programCounter];
  ++programCounter;
  currentData |= (mem[programCounter] << 8);
  ++programCounter;
  cycles -= 2;
  return currentData;
}

/** Instruction set */

/**
 * Load the content of accumulator (A register). The contents of a
 * memory location, specified by a 16-bit address in the operand, are
 * copied to the accumulator.
 */
void CPU::loadAccumulator(int &cycles, const Memory &mem) {
  auto address = fetchWord(cycles, mem);
  A = mem[address];
}

/**
 * The contents of the accumulator are copied into the memory location
 * specified by the operand.
 * This is a 3-byte instruction, the second byte specifies the low-order
 * address and the third byte specifies the high-order address.
 */
void CPU::storeAccumulator(int &cycles, Memory &mem) {
  Word address = fetchWord(cycles, mem);
  mem[address] = A;
}

/**
 * Move data from SOURCE register to DESTINATION register.
 * TODO: assert that both arguments are actually registers.
 */
void CPU::moveRegister(Byte &destination, Byte &source) {
  destination = source;
}

/**
 * Move the contents of the memory location pointed by the H-L pair to
 * the DESTINATION
 */
void CPU::moveIndirectTo(Byte &destination, const Memory &mem, int &cycles) {
  auto address = readRegisterPair(H, L);
  cycles--;
  destination = mem[address];
}

/**
 * Move the contents of SOURCE register to the memory location pointed
 * by the H-L pair.
 */
void CPU::moveIndirectFrom(const Byte &source, Memory &mem, int &cycles) {
  auto address = readRegisterPair(H, L);
  cycles--;
  mem[address] = source;
}

/**
 * Move the 8-bit(1-byte) data (provided as operand with the
 * instruction) in the DESTINATION register or memory.
 */
void CPU::moveImmediate(Byte &destination, int &cycles, const Memory &mem) {
  auto data = fetchByte(cycles, mem);
  destination = data;
}

/**
 * Load the 16-bit address into 8-bit register pair (reg1,reg2)
 */
void CPU::loadImmediate(Byte &reg1, Byte &reg2, int &cycles,
                        const Memory &mem) {
  auto data = fetchWord(cycles, mem);
  writeRegisterPair(reg1, reg2, data);
}

/**
 * Load the 16-bit address into the 16 bit register reg (currently
 * only stack pointer)
 */
void CPU::loadImmediate(Word &reg, int &cycles, const Memory &mem) {
  auto data = fetchWord(cycles, mem);
  reg = data;
}

/**
 * Transfer program control to a certain memory location
 * unconditionally.
 */
void CPU::unconditionalJump(int &cycles, const Memory &mem) {
  auto destination = fetchWord(cycles, mem);
  programCounter = mem[destination];
}

/**
 * Transfer program control to a certain memory location only when
 * zero flag is not set.
 */
void CPU::jumpIfNotZero(int &cycles, const Memory &mem) {
  if (!statusFlags.z) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the zero flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to jumpIfNotZero Transfer program control to a certain
 * memory location only when zero flag is set.
 */
void CPU::jumpIfZero(int &cycles, const Memory &mem) {
  if (statusFlags.z) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the zero flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * carry flag is not set.
 */
void CPU::jumpIfNotCarry(int &cycles, const Memory &mem) {
  if (!statusFlags.c) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the carry flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to jumpIfNotCarry Transfer program control to a certain
 * memory location only when carry flag is set.
 */
void CPU::jumpIfCarry(int &cycles, const Memory &mem) {
  if (statusFlags.c) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the carry flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * carry flag is not set.
 */
void CPU::jumpIfOddParity(int &cycles, const Memory &mem) {
  if (!statusFlags.p) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the parity flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to jumpIfOddParity. Transfer program control to a certain
 * memory location only when parity flag is set.
 */
void CPU::jumpIfEvenParity(int &cycles, const Memory &mem) {
  if (statusFlags.p) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the parity flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * sign flag is not set.
 */
void CPU::jumpIfPositive(int &cycles, const Memory &mem) {
  if (!statusFlags.s) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the sign flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to jumpIfPositive Transfer program control to a certain
 * memory location only when sign flag is set.
 */
void CPU::jumpIfMinus(int &cycles, const Memory &mem) {
  if (statusFlags.s) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the sign flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Read the Target address which would be provided to the instruction
 * as operand, and then branch out to the subroutine present at that
 * address. Similar to jump instruction but also stores the returns
 * address on the stack so that the flow can return when the next
 * return command is encountered.
 */
void CPU::unconditionalCall(int &cycles, Memory &mem) {
  auto destination = fetchWord(cycles, mem);
  auto returnAddress = mem[programCounter];
  stackPush(cycles, mem, returnAddress);
  programCounter = mem[destination];
}

/**
 * Transfer program control to a certain memory location only when
 * zero flag is not set.
 */
void CPU::callIfNotZero(int &cycles, Memory &mem) {
  if (!statusFlags.z) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the zero flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to callIfNotZero Transfer program control to a certain
 * memory location only when zero flag is set.
 */
void CPU::callIfZero(int &cycles, Memory &mem) {
  if (statusFlags.z) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the zero flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * carry flag is not set.
 */
void CPU::callIfNotCarry(int &cycles, Memory &mem) {
  if (!statusFlags.c) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the carry flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to callIfNotCarry Transfer program control to a certain
 * memory location only when carry flag is set.
 */
void CPU::callIfCarry(int &cycles, Memory &mem) {
  if (statusFlags.c) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the carry flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * carry flag is not set.
 */
void CPU::callIfOddParity(int &cycles, Memory &mem) {
  if (!statusFlags.p) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the parity flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to callIfOddParity. Transfer program control to a certain
 * memory location only when parity flag is set.
 */
void CPU::callIfEvenParity(int &cycles, Memory &mem) {
  if (statusFlags.p) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the parity flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * sign flag is not set.
 */
void CPU::callIfPositive(int &cycles, Memory &mem) {
  if (!statusFlags.s) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the sign flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to callIfPositive Transfer program control to a certain
 * memory location only when sign flag is set.
 */
void CPU::callIfMinus(int &cycles, Memory &mem) {
  if (statusFlags.s) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the sign flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Push the target on top os stack ( pointed by the stack pointer
 * ). As the stack grows downwards in this processor, decrement the
 * stack pointer to the new top
 */
void CPU::stackPush(int &cycles, Memory &mem, const Byte &data) {
  cycles++;
  mem[stackPointer] = data;
  stackPointer--;
}

/**
 * Pop and return the data on top of the stack (pointed by the stack
 * pointer). As the satck grows downwards in this processor, increment
 * the stack pointer to the new top.
 */
Byte CPU::stackPop(int &cycles, const Memory &mem) {
  cycles++;
  auto data = mem[stackPointer];
  stackPointer++;
  return data;
}

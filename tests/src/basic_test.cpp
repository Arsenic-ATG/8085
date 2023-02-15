#include "cpu.h"
#include <gtest/gtest.h>

class emulatorTest1 : public testing::Test {
public:
  emu::Memory mem;
  emu::CPU cpu;

  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(emulatorTest1, CPUinitialisationAndMemoryReset) {
  cpu.reset(mem);
  EXPECT_EQ(cpu.getProgramCounter(), 0);
}

#include <unity.h>

#include "Device/Wk2132Protocol.hpp"

using namespace Espfc::Device::Wk2132Protocol;

void setUp() {}
void tearDown() {}

void test_i2c_addresses_match_wk2132_layout()
{
  TEST_ASSERT_EQUAL_HEX8(0x70, i2cAddress(true, true, 0, false));
  TEST_ASSERT_EQUAL_HEX8(0x71, i2cAddress(true, true, 0, true));
  TEST_ASSERT_EQUAL_HEX8(0x72, i2cAddress(true, true, 1, false));
  TEST_ASSERT_EQUAL_HEX8(0x73, i2cAddress(true, true, 1, true));
  TEST_ASSERT_EQUAL_HEX8(0x10, i2cAddress(false, false, 0, false));
}

void test_july_28_pcb_addresses_and_channel_wiring()
{
  TEST_ASSERT_EQUAL_UINT8(0, CHANNEL_UWB);
  TEST_ASSERT_EQUAL_UINT8(1, CHANNEL_CAMERA);
  TEST_ASSERT_EQUAL_HEX8(0x30, i2cAddress(false, true, CHANNEL_UWB, false));
  TEST_ASSERT_EQUAL_HEX8(0x31, i2cAddress(false, true, CHANNEL_UWB, true));
  TEST_ASSERT_EQUAL_HEX8(0x32, i2cAddress(false, true, CHANNEL_CAMERA, false));
  TEST_ASSERT_EQUAL_HEX8(0x33, i2cAddress(false, true, CHANNEL_CAMERA, true));
}

void test_common_crystals_produce_exact_115200_baud()
{
  const BaudConfig crystal147456 = calculateBaud(14745600, 115200);
  TEST_ASSERT_TRUE(crystal147456.valid);
  TEST_ASSERT_EQUAL_HEX16(0x0007, crystal147456.divisorRegister);
  TEST_ASSERT_EQUAL_UINT8(0, crystal147456.prescaler);
  TEST_ASSERT_EQUAL_UINT32(115200, crystal147456.actualBaud);
  TEST_ASSERT_EQUAL_UINT32(0, crystal147456.errorPpm);

  const BaudConfig crystal110592 = calculateBaud(11059200, 115200);
  TEST_ASSERT_TRUE(crystal110592.valid);
  TEST_ASSERT_EQUAL_HEX16(0x0005, crystal110592.divisorRegister);
  TEST_ASSERT_EQUAL_UINT8(0, crystal110592.prescaler);
  TEST_ASSERT_EQUAL_UINT32(115200, crystal110592.actualBaud);
}

void test_fractional_prescaler_is_rounded()
{
  const BaudConfig baud = calculateBaud(12000000, 115200);
  TEST_ASSERT_TRUE(baud.valid);
  TEST_ASSERT_EQUAL_HEX16(0x0005, baud.divisorRegister);
  TEST_ASSERT_EQUAL_UINT8(8, baud.prescaler);
  TEST_ASSERT_EQUAL_UINT32(115384, baud.actualBaud);
  TEST_ASSERT_LESS_THAN_UINT32(2000, baud.errorPpm);
}

void test_unreachable_baud_is_rejected()
{
  TEST_ASSERT_FALSE(calculateBaud(14745600, 2000000).valid);
  TEST_ASSERT_FALSE(calculateBaud(0, 115200).valid);
  TEST_ASSERT_FALSE(calculateBaud(14745600, 0).valid);
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_i2c_addresses_match_wk2132_layout);
  RUN_TEST(test_july_28_pcb_addresses_and_channel_wiring);
  RUN_TEST(test_common_crystals_produce_exact_115200_baud);
  RUN_TEST(test_fractional_prescaler_is_rounded);
  RUN_TEST(test_unreachable_baud_is_rejected);
  return UNITY_END();
}

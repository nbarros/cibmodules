/**
 * @file CIBUtilities_test.cxx
 *
 * Unit tests for CIB utility functions (bitmask operations, signed conversions, etc.)
 *
 * This is part of the DUNE DAQ Application Framework, copyright 2020.
 * Licensing/copyright details are in the COPYING file that you should have
 * received with this code.
 */

#define BOOST_TEST_MODULE CIBUtilities_test // NOLINT

#include "boost/test/unit_test.hpp"

#include <cstdint>

// Replicate utility functions from CIBModule.cpp for testing
namespace test_util {

  uint32_t bitmask(uint32_t highbit, uint32_t lowbit)
  {
    // sort the bit order or this fails miserably
    if (highbit < lowbit)
    {
      uint32_t tmp = lowbit;
      lowbit = highbit;
      highbit = tmp;
    }

    uint32_t i = ~0U;
    return ~(i << highbit << 1) & (i << lowbit);
  }
  
  int32_t cast_to_signed(const uint32_t reg, const uint32_t mask)
  {
    // first find the msb in the mask. That will be the signed bit
    uint32_t msb = 0;
    int32_t res = 0;
    for (size_t bit = 31; bit > 0; bit--)
    {
      if ((1U << bit) & mask)
      {
        msb = bit;
        break;
      }
    }
    
    // check the msb of the register. That is the sign bit
    if ((1U << msb) & reg)
    {
      res = bitmask(31, msb + 1); // set all bits to 1 above the mask
      res = res | (reg & mask);
    }
    else
    {
      res = (reg & mask);
    }
    return res;
  }

} // namespace test_util

BOOST_AUTO_TEST_SUITE(CIBUtilities_test)

// Test bitmask generation
BOOST_AUTO_TEST_CASE(TestBitmaskGeneration)
{
  BOOST_TEST_MESSAGE("Testing bitmask utility function");
  
  // Test single bit mask
  BOOST_CHECK_EQUAL(test_util::bitmask(0, 0), 0x1);
  BOOST_CHECK_EQUAL(test_util::bitmask(1, 1), 0x2);
  
  // Test range masks
  BOOST_CHECK_EQUAL(test_util::bitmask(3, 0), 0xF);      // bits 0-3: 0b1111
  BOOST_CHECK_EQUAL(test_util::bitmask(7, 0), 0xFF);     // bits 0-7: 0b11111111
  BOOST_CHECK_EQUAL(test_util::bitmask(15, 0), 0xFFFF);  // bits 0-15
  
  // Test offset masks
  BOOST_CHECK_EQUAL(test_util::bitmask(7, 4), 0xF0);     // bits 4-7: 0b11110000
  BOOST_CHECK_EQUAL(test_util::bitmask(11, 8), 0xF00);   // bits 8-11
  
  // Test reversed arguments (should handle correctly)
  BOOST_CHECK_EQUAL(test_util::bitmask(0, 3), 0xF);      // swaps to (3, 0)
  BOOST_CHECK_EQUAL(test_util::bitmask(4, 7), 0xF0);     // swaps to (7, 4)
}

// Test bitmask for motor position masks
BOOST_AUTO_TEST_CASE(TestMotorPositionBitmasks)
{
  BOOST_TEST_MESSAGE("Testing bitmask for motor position bit ranges");
  
  // M1: 22 bits (bitmask should be 0x3FFFFF)
  BOOST_CHECK_EQUAL(test_util::bitmask(21, 0), 0x3FFFFF);
  
  // M2: 22 bits (bitmask should be 0x3FFFFF)
  BOOST_CHECK_EQUAL(test_util::bitmask(21, 0), 0x3FFFFF);
  
  // M3: 17 bits (bitmask should be 0x1FFFF)
  BOOST_CHECK_EQUAL(test_util::bitmask(16, 0), 0x1FFFF);
}

// Test signed conversion for positive values
BOOST_AUTO_TEST_CASE(TestSignedConversionPositive)
{
  BOOST_TEST_MESSAGE("Testing signed conversion for positive values");
  
  // Small positive value with 8-bit mask
  uint32_t mask_8bit = 0xFF;
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x7F, mask_8bit), 0x7F);  // +127
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x01, mask_8bit), 0x01);  // +1
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x00, mask_8bit), 0x00);  // 0
  
  // Positive value with 16-bit mask
  uint32_t mask_16bit = 0xFFFF;
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x7FFF, mask_16bit), 0x7FFF);  // +32767
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x1234, mask_16bit), 0x1234);  // +4660
}

// Test signed conversion for negative values
BOOST_AUTO_TEST_CASE(TestSignedConversionNegative)
{
  BOOST_TEST_MESSAGE("Testing signed conversion for negative values");
  
  // Negative value with 8-bit mask
  uint32_t mask_8bit = 0xFF;
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x80, mask_8bit), -128);  // Most negative 8-bit
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0xFF, mask_8bit), -1);    // -1 in 8-bit
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0xFE, mask_8bit), -2);    // -2 in 8-bit
  
  // Negative value with 16-bit mask
  uint32_t mask_16bit = 0xFFFF;
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x8000, mask_16bit), -32768);  // Most negative 16-bit
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0xFFFF, mask_16bit), -1);      // -1 in 16-bit
}

// Test signed conversion for motor position masks (22-bit)
BOOST_AUTO_TEST_CASE(TestSignedConversionMotor22bit)
{
  BOOST_TEST_MESSAGE("Testing signed conversion with 22-bit motor mask");
  
  uint32_t mask_22bit = 0x3FFFFF;
  
  // Positive values
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x000000, mask_22bit), 0);
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x000001, mask_22bit), 1);
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x1FFFFF, mask_22bit), 0x1FFFFF);  // Max positive
  
  // Negative values (MSB set in 22-bit field)
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x200000, mask_22bit), -2097152);  // Most negative
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x3FFFFF, mask_22bit), -1);        // -1
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x3FFFFE, mask_22bit), -2);        // -2
}

// Test signed conversion for motor position masks (17-bit M3)
BOOST_AUTO_TEST_CASE(TestSignedConversionMotor17bit)
{
  BOOST_TEST_MESSAGE("Testing signed conversion with 17-bit motor mask (M3)");
  
  uint32_t mask_17bit = 0x1FFFF;
  
  // Positive values
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x00000, mask_17bit), 0);
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x00001, mask_17bit), 1);
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x0FFFF, mask_17bit), 0x0FFFF);  // Max positive
  
  // Negative values (MSB set in 17-bit field)
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x10000, mask_17bit), -65536);   // Most negative
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x1FFFF, mask_17bit), -1);       // -1
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x1FFFE, mask_17bit), -2);       // -2
}

// Test edge cases for signed conversion
BOOST_AUTO_TEST_CASE(TestSignedConversionEdgeCases)
{
  BOOST_TEST_MESSAGE("Testing signed conversion edge cases");
  
  // Test mask with single bit set
  uint32_t mask_1bit = 0x1;
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x0, mask_1bit), 0);
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x1, mask_1bit), -1);  // Sign bit set
  
  // Test non-contiguous mask (should still work based on MSB)
  uint32_t mask_sparse = 0x101;  // bits 0 and 8
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x001, mask_sparse), 0x001);   // Positive
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x101, mask_sparse), -255);    // Negative (bit 8 set)
}

// Test realistic motor position scenarios
BOOST_AUTO_TEST_CASE(TestRealisticMotorPositions)
{
  BOOST_TEST_MESSAGE("Testing realistic motor position value conversions");
  
  uint32_t mask_m1_m2 = 0x3FFFFF;  // 22-bit mask for M1 and M2
  uint32_t mask_m3 = 0x1FFFF;      // 17-bit mask for M3
  
  // Test typical positive positions
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(10000, mask_m1_m2), 10000);
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(5000, mask_m3), 5000);
  
  // Test typical negative positions (motors can move in both directions)
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x3FD8F0, mask_m1_m2), -10000);  // -10000 in 22-bit
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0x1EC78, mask_m3), -5000);       // -5000 in 17-bit
  
  // Test center position (zero)
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0, mask_m1_m2), 0);
  BOOST_CHECK_EQUAL(test_util::cast_to_signed(0, mask_m3), 0);
}

BOOST_AUTO_TEST_SUITE_END()

/**
 * @file MotorPositionExtraction_test.cxx
 *
 * Comprehensive tests for motor position extraction from IoLS trigger packets
 * Tests the complete pipeline: bitfield → unsigned → signed conversion
 *
 * This is part of the DUNE DAQ Application Framework, copyright 2020.
 * Licensing/copyright details are in the COPYING file that you should have
 * received with this code.
 */

#define BOOST_TEST_MODULE MotorPositionExtraction_test // NOLINT

#include "boost/test/unit_test.hpp"

#define CIB_DUNEDAQ 1
#include "cib_data_fmt.h"
#include <cstdint>

using namespace dunedaq::cib::daq;

// Replicate utility functions from CIBModule.cpp for testing
namespace extraction_util {

  uint32_t bitmask(uint32_t highbit, uint32_t lowbit)
  {
    if (highbit < lowbit) {
      uint32_t tmp = lowbit;
      lowbit = highbit;
      highbit = tmp;
    }
    uint32_t i = ~0U;
    return ~(i << highbit << 1) & (i << lowbit);
  }
  
  int32_t cast_to_signed(const uint32_t reg, const uint32_t mask)
  {
    uint32_t msb = 0;
    int32_t res = 0;
    for (size_t bit = 31; bit > 0; bit--) {
      if ((1U << bit) & mask) {
        msb = bit;
        break;
      }
    }
    
    if ((1U << msb) & reg) {
      res = bitmask(31, msb + 1);
      res = res | (reg & mask);
    } else {
      res = (reg & mask);
    }
    return res;
  }

  int32_t get_m1(iols_trigger_t &t) {
    return cast_to_signed(t.pos_m1, t.bitmask_m1);
  }

  int32_t get_m2(iols_trigger_t &t) {
    uint32_t m2_lsb = t.pos_m2_lsb;
    uint32_t m2_msb = t.pos_m2_msb;
    uint32_t m2 = (m2_msb << 15) | t.pos_m2_lsb;
    return cast_to_signed(m2, t.bitmask_m2);
  }

  int32_t get_m3(iols_trigger_t &t) {
    return cast_to_signed(t.pos_m3, t.bitmask_m3);
  }

} // namespace extraction_util

BOOST_AUTO_TEST_SUITE(MotorPositionExtraction_test)

// Test M1 position extraction (22-bit signed)
BOOST_AUTO_TEST_CASE(TestM1PositionExtraction)
{
  BOOST_TEST_MESSAGE("Testing M1 motor position extraction");
  
  iols_trigger_t trigger;
  trigger.timestamp = 0;
  trigger.padding = 0;
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 0;
  trigger.pos_m3 = 0;
  
  // Test zero position
  trigger.pos_m1 = 0;
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), 0);
  
  // Test positive positions
  trigger.pos_m1 = 1;
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), 1);
  
  trigger.pos_m1 = 1000;
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), 1000);
  
  trigger.pos_m1 = 100000;
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), 100000);
  
  // Test maximum positive (bit 21 clear, all others set)
  trigger.pos_m1 = 0x1FFFFF;  // Max positive in 22-bit signed
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), 0x1FFFFF);
  
  // Test negative positions (bit 21 set = sign bit)
  trigger.pos_m1 = 0x3FFFFF;  // All bits set = -1
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), -1);
  
  trigger.pos_m1 = 0x3FFFFE;  // -2
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), -2);
  
  trigger.pos_m1 = 0x200000;  // Most negative 22-bit value
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), -2097152);
}

// Test M2 position extraction (22-bit signed, split across two fields)
BOOST_AUTO_TEST_CASE(TestM2PositionExtraction)
{
  BOOST_TEST_MESSAGE("Testing M2 motor position extraction (split field)");
  
  iols_trigger_t trigger;
  trigger.timestamp = 0;
  trigger.padding = 0;
  trigger.pos_m1 = 0;
  trigger.pos_m3 = 0;
  
  // Test zero position
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 0;
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), 0);
  
  // Test positive position with only LSB
  trigger.pos_m2_lsb = 1000;
  trigger.pos_m2_msb = 0;
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), 1000);
  
  // Test positive position with only MSB
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 5;  // MSB contributes (5 << 15) = 163840
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), 163840);
  
  // Test positive position with both fields
  trigger.pos_m2_lsb = 12345;
  trigger.pos_m2_msb = 10;  // Contributes (10 << 15) = 327680
  int32_t expected = 327680 + 12345;
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), expected);
  
  // Test maximum positive (MSB = 63 max for 22-bit positive)
  trigger.pos_m2_lsb = 0x7FFF;  // All 15 LSB bits set
  trigger.pos_m2_msb = 0x3F;    // 6 bits (leaving sign bit clear)
  // Result: (0x3F << 15) | 0x7FFF = 0x1FFFFF (max positive 22-bit)
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), 0x1FFFFF);
  
  // Test negative position (MSB bit 6 set = sign bit in combined value)
  trigger.pos_m2_lsb = 0x7FFF;
  trigger.pos_m2_msb = 0x7F;  // All 7 bits set
  // Result: (0x7F << 15) | 0x7FFF = 0x3FFFFF = -1 in 22-bit signed
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), -1);
  
  // Test another negative
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 0x40;  // Sign bit set, others clear
  // Result: 0x40 << 15 = 0x200000 = most negative 22-bit value
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), -2097152);
}

// Test M3 position extraction (17-bit signed)
BOOST_AUTO_TEST_CASE(TestM3PositionExtraction)
{
  BOOST_TEST_MESSAGE("Testing M3 motor position extraction");
  
  iols_trigger_t trigger;
  trigger.timestamp = 0;
  trigger.padding = 0;
  trigger.pos_m1 = 0;
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 0;
  
  // Test zero position
  trigger.pos_m3 = 0;
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), 0);
  
  // Test positive positions
  trigger.pos_m3 = 1;
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), 1);
  
  trigger.pos_m3 = 5000;
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), 5000);
  
  trigger.pos_m3 = 30000;
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), 30000);
  
  // Test maximum positive (bit 16 clear, all others set)
  trigger.pos_m3 = 0x0FFFF;  // Max positive in 17-bit signed
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), 0x0FFFF);
  
  // Test negative positions (bit 16 set = sign bit)
  trigger.pos_m3 = 0x1FFFF;  // All bits set = -1
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), -1);
  
  trigger.pos_m3 = 0x1FFFE;  // -2
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), -2);
  
  trigger.pos_m3 = 0x10000;  // Most negative 17-bit value
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), -65536);
}

// Test all three motors together in realistic scenario
BOOST_AUTO_TEST_CASE(TestAllMotorsRealisticScenario)
{
  BOOST_TEST_MESSAGE("Testing realistic multi-motor position scenario");
  
  iols_trigger_t trigger;
  trigger.timestamp = 1234567890ULL;
  trigger.padding = 0;
  
  // Scenario: M1 at +50000, M2 at -10000, M3 at +25000
  trigger.pos_m1 = 50000;
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), 50000);
  
  // M2 = -10000 in 22-bit signed
  // -10000 as unsigned 22-bit: 0x3FFFFF - 10000 + 1 = 0x3FD8F0
  uint32_t m2_unsigned = 0x3FD8F0;
  trigger.pos_m2_lsb = m2_unsigned & 0x7FFF;        // Lower 15 bits
  trigger.pos_m2_msb = (m2_unsigned >> 15) & 0x7F;  // Upper 7 bits
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), -10000);
  
  trigger.pos_m3 = 25000;
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), 25000);
  
  // Verify timestamp preserved
  BOOST_CHECK_EQUAL(trigger.timestamp, 1234567890ULL);
}

// Test motor position symmetry (positive and negative magnitudes)
BOOST_AUTO_TEST_CASE(TestMotorPositionSymmetry)
{
  BOOST_TEST_MESSAGE("Testing motor position symmetry for positive/negative values");
  
  iols_trigger_t trigger;
  trigger.timestamp = 0;
  trigger.padding = 0;
  
  // Test M1 symmetry
  trigger.pos_m1 = 12345;
  int32_t m1_pos = extraction_util::get_m1(trigger);
  BOOST_CHECK_EQUAL(m1_pos, 12345);
  
  // Convert to negative: 2's complement in 22-bit
  trigger.pos_m1 = (~12345 + 1) & 0x3FFFFF;
  int32_t m1_neg = extraction_util::get_m1(trigger);
  BOOST_CHECK_EQUAL(m1_neg, -12345);
  
  // Test M3 symmetry (smaller range)
  trigger.pos_m3 = 5678;
  int32_t m3_pos = extraction_util::get_m3(trigger);
  BOOST_CHECK_EQUAL(m3_pos, 5678);
  
  trigger.pos_m3 = (~5678 + 1) & 0x1FFFF;
  int32_t m3_neg = extraction_util::get_m3(trigger);
  BOOST_CHECK_EQUAL(m3_neg, -5678);
}

// Test edge case: all motors at maximum positive
BOOST_AUTO_TEST_CASE(TestAllMotorsMaxPositive)
{
  BOOST_TEST_MESSAGE("Testing all motors at maximum positive positions");
  
  iols_trigger_t trigger;
  trigger.timestamp = 0xFFFFFFFFFFFFFFFFULL;
  trigger.padding = 0;
  
  // M1 max positive: 2^21 - 1
  trigger.pos_m1 = 0x1FFFFF;
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), 2097151);
  
  // M2 max positive: 2^21 - 1 (22-bit signed)
  trigger.pos_m2_lsb = 0x7FFF;
  trigger.pos_m2_msb = 0x3F;
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), 2097151);
  
  // M3 max positive: 2^16 - 1
  trigger.pos_m3 = 0x0FFFF;
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), 65535);
}

// Test edge case: all motors at maximum negative
BOOST_AUTO_TEST_CASE(TestAllMotorsMaxNegative)
{
  BOOST_TEST_MESSAGE("Testing all motors at maximum negative positions");
  
  iols_trigger_t trigger;
  trigger.timestamp = 0;
  trigger.padding = 0;
  
  // M1 max negative: -2^21
  trigger.pos_m1 = 0x200000;
  BOOST_CHECK_EQUAL(extraction_util::get_m1(trigger), -2097152);
  
  // M2 max negative: -2^21
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 0x40;
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), -2097152);
  
  // M3 max negative: -2^16
  trigger.pos_m3 = 0x10000;
  BOOST_CHECK_EQUAL(extraction_util::get_m3(trigger), -65536);
}

// Test M2 LSB/MSB boundary crossing
BOOST_AUTO_TEST_CASE(TestM2BoundaryCrossing)
{
  BOOST_TEST_MESSAGE("Testing M2 position across LSB/MSB boundary");
  
  iols_trigger_t trigger;
  trigger.timestamp = 0;
  trigger.padding = 0;
  trigger.pos_m1 = 0;
  trigger.pos_m3 = 0;
  
  // Test at LSB boundary (15-bit overflow point)
  // Value = 0x7FFF (all LSB bits set)
  trigger.pos_m2_lsb = 0x7FFF;
  trigger.pos_m2_msb = 0;
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), 32767);
  
  // Increment by 1 (should roll into MSB)
  // Value = 0x8000 = (1 << 15)
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 1;
  BOOST_CHECK_EQUAL(extraction_util::get_m2(trigger), 32768);
  
  // Test around sign bit boundary
  // Value = 0x1FFFFF (max positive, sign bit clear)
  trigger.pos_m2_lsb = 0x7FFF;
  trigger.pos_m2_msb = 0x3F;
  int32_t max_pos = extraction_util::get_m2(trigger);
  BOOST_CHECK_GT(max_pos, 0);
  
  // Value = 0x200000 (sign bit set, min negative)
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 0x40;
  int32_t min_neg = extraction_util::get_m2(trigger);
  BOOST_CHECK_LT(min_neg, 0);
}

BOOST_AUTO_TEST_SUITE_END()

/**
 * @file CIBDataFormat_test.cxx
 *
 * Unit tests for CIB data format structures and parsing utilities
 *
 * This is part of the DUNE DAQ Application Framework, copyright 2020.
 * Licensing/copyright details are in the COPYING file that you should have
 * received with this code.
 */

#define BOOST_TEST_MODULE CIBDataFormat_test // NOLINT

#include "boost/test/unit_test.hpp"

#define CIB_DUNEDAQ 1
#include "cib_data_fmt.h"
#include <cstdint>
#include <limits>

using namespace dunedaq::cib::daq;

BOOST_AUTO_TEST_SUITE(CIBDataFormat_test)

// Test data structure sizes
BOOST_AUTO_TEST_CASE(TestDataStructureSizes)
{
  BOOST_TEST_MESSAGE("Testing CIB data structure sizes");
  
  // iols_trigger_t should be 16 bytes (8 bytes position data + 8 bytes timestamp)
  BOOST_CHECK_EQUAL(sizeof(iols_trigger_t), 16);
  BOOST_CHECK_EQUAL(iols_trigger_t::size_bytes, 16);
  
  // tcp_header_t should be 4 bytes
  BOOST_CHECK_EQUAL(sizeof(tcp_header_t), 4);
  BOOST_CHECK_EQUAL(tcp_header_t::size_bytes, 4);
  
  // iols_tcp_packet_t should be header + trigger = 20 bytes
  BOOST_CHECK_EQUAL(sizeof(iols_tcp_packet_t), 20);
}

// Test bit field packing and masks
BOOST_AUTO_TEST_CASE(TestTriggerBitfieldPacking)
{
  BOOST_TEST_MESSAGE("Testing iols_trigger_t bitfield packing");
  
  iols_trigger_t trigger;
  
  // Zero initialization
  trigger.pos_m3 = 0;
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 0;
  trigger.pos_m1 = 0;
  trigger.padding = 0;
  trigger.timestamp = 0;
  
  // Test maximum values fit in bit fields
  trigger.pos_m3 = (1 << 17) - 1;     // 17 bits
  BOOST_CHECK_EQUAL(trigger.pos_m3, 0x1FFFF);
  
  trigger.pos_m2_lsb = (1 << 15) - 1; // 15 bits
  BOOST_CHECK_EQUAL(trigger.pos_m2_lsb, 0x7FFF);
  
  trigger.pos_m2_msb = (1 << 7) - 1;  // 7 bits
  BOOST_CHECK_EQUAL(trigger.pos_m2_msb, 0x7F);
  
  trigger.pos_m1 = (1 << 22) - 1;     // 22 bits
  BOOST_CHECK_EQUAL(trigger.pos_m1, 0x3FFFFF);
}

// Test bitmask constants
BOOST_AUTO_TEST_CASE(TestBitmaskConstants)
{
  BOOST_TEST_MESSAGE("Testing iols_trigger_t bitmask constants");
  
  // Check that bitmasks match expected bit widths
  BOOST_CHECK_EQUAL(iols_trigger_t::bitmask_m3, 0x1FFFF);   // 17 bits
  BOOST_CHECK_EQUAL(iols_trigger_t::bitmask_m2, 0x3FFFFF);  // 22 bits  
  BOOST_CHECK_EQUAL(iols_trigger_t::bitmask_m1, 0x3FFFFF);  // 22 bits
}

// Test TCP header version encoding/decoding
BOOST_AUTO_TEST_CASE(TestTCPHeaderVersion)
{
  BOOST_TEST_MESSAGE("Testing TCP header version manipulation");
  
  tcp_header_t header;
  header.packet_size = 0;
  header.sequence_id = 0;
  header.format_version = 0;
  
  // Test version setting and getting
  header.set_version(1);
  BOOST_CHECK_EQUAL(header.get_version(), 1);
  
  header.set_version(5);
  BOOST_CHECK_EQUAL(header.get_version(), 5);
  
  header.set_version(15); // Max 4-bit value
  BOOST_CHECK_EQUAL(header.get_version(), 15);
}

// Test TCP header fields
BOOST_AUTO_TEST_CASE(TestTCPHeaderFields)
{
  BOOST_TEST_MESSAGE("Testing TCP header field assignments");
  
  tcp_header_t header;
  
  // Test packet size (16 bits)
  header.packet_size = 1234;
  BOOST_CHECK_EQUAL(header.packet_size, 1234);
  
  header.packet_size = 0xFFFF;
  BOOST_CHECK_EQUAL(header.packet_size, 0xFFFF);
  
  // Test sequence ID (8 bits, wraps at 256)
  header.sequence_id = 127;
  BOOST_CHECK_EQUAL(header.sequence_id, 127);
  
  header.sequence_id = 255;
  BOOST_CHECK_EQUAL(header.sequence_id, 255);
}

// Test TCP packet structure
BOOST_AUTO_TEST_CASE(TestTCPPacketStructure)
{
  BOOST_TEST_MESSAGE("Testing iols_tcp_packet_t structure");
  
  iols_tcp_packet_t packet;
  
  // Set header values
  packet.header.packet_size = sizeof(iols_trigger_t);
  packet.header.sequence_id = 42;
  packet.header.set_version(1);
  
  // Set trigger values
  packet.word.pos_m1 = 1000;
  packet.word.pos_m2_lsb = 2000;
  packet.word.pos_m2_msb = 50;
  packet.word.pos_m3 = 3000;
  packet.word.timestamp = 123456789ULL;
  
  // Verify values persist
  BOOST_CHECK_EQUAL(packet.header.packet_size, 16);
  BOOST_CHECK_EQUAL(packet.header.sequence_id, 42);
  BOOST_CHECK_EQUAL(packet.header.get_version(), 1);
  BOOST_CHECK_EQUAL(packet.word.pos_m1, 1000);
  BOOST_CHECK_EQUAL(packet.word.timestamp, 123456789ULL);
}

// Test timestamp handling (64-bit unsigned)
BOOST_AUTO_TEST_CASE(TestTimestampHandling)
{
  BOOST_TEST_MESSAGE("Testing timestamp field handling");
  
  iols_trigger_t trigger;
  
  // Test small timestamp
  trigger.timestamp = 0;
  BOOST_CHECK_EQUAL(trigger.timestamp, 0);
  
  // Test medium timestamp
  trigger.timestamp = 1000000000ULL;
  BOOST_CHECK_EQUAL(trigger.timestamp, 1000000000ULL);
  
  // Test large timestamp (close to max 64-bit)
  trigger.timestamp = 0xFFFFFFFFFFFFFFFFULL;
  BOOST_CHECK_EQUAL(trigger.timestamp, 0xFFFFFFFFFFFFFFFFULL);
}

// Test motor position edge cases
BOOST_AUTO_TEST_CASE(TestMotorPositionEdgeCases)
{
  BOOST_TEST_MESSAGE("Testing motor position boundary values");
  
  iols_trigger_t trigger;
  
  // Test zero positions
  trigger.pos_m1 = 0;
  trigger.pos_m2_lsb = 0;
  trigger.pos_m2_msb = 0;
  trigger.pos_m3 = 0;
  
  BOOST_CHECK_EQUAL(trigger.pos_m1, 0);
  BOOST_CHECK_EQUAL(trigger.pos_m2_lsb, 0);
  BOOST_CHECK_EQUAL(trigger.pos_m2_msb, 0);
  BOOST_CHECK_EQUAL(trigger.pos_m3, 0);
  
  // Test maximum positions (signed values will be tested in utility function tests)
  trigger.pos_m1 = (1 << 22) - 1;
  trigger.pos_m3 = (1 << 17) - 1;
  
  BOOST_CHECK_EQUAL(trigger.pos_m1, 0x3FFFFF);
  BOOST_CHECK_EQUAL(trigger.pos_m3, 0x1FFFF);
}

BOOST_AUTO_TEST_SUITE_END()

# CIBModules Unit Tests

This directory contains unit tests for the CIBModules package.

## Test Files

### 1. CIBDataFormat_test.cxx
Tests the CIB data format structures and binary layout.

**Test Coverage:**
- `iols_trigger_t` structure size and alignment (16 bytes expected)
- `tcp_header_t` structure size (4 bytes expected)
- `iols_tcp_packet_t` structure size (20 bytes expected)
- Bitfield packing for motor positions (m1: 22-bit, m2: 22-bit split, m3: 17-bit)
- Bitmask constants validation
- TCP header version encoding/decoding
- Timestamp field handling (64-bit unsigned)
- Motor position boundary values

**Purpose:** Ensures binary data structures match hardware expectations and are correctly packed for network transmission.

### 2. CIBUtilities_test.cxx
Tests utility functions used for motor position parsing and bit manipulation.

**Test Coverage:**
- `bitmask()` function - generates bit masks for field extraction
- `cast_to_signed()` function - converts masked unsigned values to signed integers
- Motor position conversion for all three motors (M1, M2, M3)
- Positive and negative position values
- Edge cases (zero, maximum positive/negative, boundary conditions)
- Realistic motor position scenarios

**Purpose:** Validates the mathematical correctness of motor position parsing from bitfields to signed integer positions.

### 3. Placeholder_test.cxx (renamed to CIBModules_Basic_test)
Tests basic infrastructure and utility patterns used in CIBModule.

**Test Coverage:**
- Basic test infrastructure sanity check
- Hex string parsing (used for trigger bit configuration)
- Buffer count averaging logic
- Atomic flag operations (state management)
- Atomic counter operations (metrics tracking)
- Message type string detection (error/warning/info classification)

**Purpose:** Validates common patterns and utilities used throughout the CIBModule implementation without requiring full DAQ framework initialization.

### 4. MotorPositionExtraction_test.cxx
Comprehensive tests for the complete motor position extraction pipeline.

**Test Coverage:**
- M1 motor position extraction (22-bit signed)
- M2 motor position extraction (22-bit signed, split across LSB/MSB fields)
- M3 motor position extraction (17-bit signed)
- Zero, positive, and negative position values for all motors
- Maximum positive and negative bounds for each motor
- Multi-motor realistic scenarios with mixed signs
- Position symmetry (positive/negative magnitude equivalence)
- M2 LSB/MSB boundary crossing and concatenation
- Timestamp preservation across motor data

**Purpose:** Ensures critical motor position data is correctly extracted from binary packets. This is essential for laser calibration system accuracy.

## Running Tests

### Build and Run All Tests
```bash
cd build
cmake ..
make
ctest --output-on-failure
```

### Run Specific Test Suite
```bash
cd build
./unittest/CIBDataFormat_test --log_level=all
./unittest/CIBUtilities_test --log_level=all
./unittest/CIBModules_Basic_test --log_level=all
./unittest/MotorPositionExtraction_test --log_level=all
```

### Run Specific Test Case
```bash
./unittest/CIBDataFormat_test --run_test=CIBDataFormat_test/TestDataStructureSizes
./unittest/MotorPositionExtraction_test --run_test=MotorPositionExtraction_test/TestM1PositionExtraction
./unittest/CIBUtilities_test --run_test=CIBUtilities_test/TestBitmaskGeneration
```

## Test Framework

All tests use **Boost.Test** framework (https://www.boost.org/doc/libs/release/libs/test/)

Key macros used:
- `BOOST_AUTO_TEST_SUITE` / `BOOST_AUTO_TEST_SUITE_END` - Define test suite
- `BOOST_AUTO_TEST_CASE` - Define individual test case
- `BOOST_CHECK_EQUAL(a, b)` - Assert equality (continues on failure)
- `BOOST_REQUIRE_EQUAL(a, b)` - Assert equality (stops on failure)
- `BOOST_CHECK(condition)` - Assert condition is true
- `BOOST_TEST_MESSAGE(msg)` - Log test progress message

## Coverage Summary

| Component | Test File | Coverage |
|-----------|-----------|----------|
| Data Structures | CIBDataFormat_test.cxx | ✅ Complete |
| Motor Position Extraction | MotorPositionExtraction_test.cxx | ✅ Complete |
| Utility Functions | CIBUtilities_test.cxx | ✅ Complete |
| Basic Infrastructure | CIBModules_Basic_test.cxx | ✅ Complete |
| Network/Socket Operations | - | ⚠️ Not covered (requires mocking) |
| DAQ Framework Integration | - | ⚠️ Not covered (requires full DAQ) |
| Hardware Communication | - | ⚠️ Not covered (hardware-dependent) |

## Future Test Additions

Potential areas for additional testing:

1. **Mock Socket Tests**: Use mock sockets to test network communication patterns
2. **State Machine Tests**: Test full lifecycle state transitions (configure → start → stop)
3. **HSI Frame Generation**: Test HSI frame construction from trigger data
4. **Configuration Parsing**: Test appmodel configuration object parsing
5. **Error Handling**: Test exception throwing and ERS issue generation
6. **Thread Synchronization**: Test worker thread lifecycle and coordination
7. **Calibration File I/O**: Test calibration stream file handling

## Adding New Tests

To add new tests:

1. Create a new `.cxx` file in this directory with name ending in `_test.cxx`
2. Use `#define BOOST_TEST_MODULE YourTestName` at the top
3. Include necessary headers
4. Wrap tests in `BOOST_AUTO_TEST_SUITE(YourTestName)` / `BOOST_AUTO_TEST_SUITE_END()`
5. Define test cases with `BOOST_AUTO_TEST_CASE(TestCaseName)`
6. The daq-cmake framework will automatically discover and compile your tests

Example:
```cpp
#define BOOST_TEST_MODULE MyNewTest
#include "boost/test/unit_test.hpp"

BOOST_AUTO_TEST_SUITE(MyNewTest)

BOOST_AUTO_TEST_CASE(MyTestCase)
{
  BOOST_TEST_MESSAGE("Testing something");
  BOOST_CHECK_EQUAL(2 + 2, 4);
}

BOOST_AUTO_TEST_SUITE_END()
```

## Dependencies

Tests depend on:
- Boost.Test (provided by Boost library)
- CIB data format headers (`cib_data_fmt.h`)
- Standard C++ library
- No DAQ framework dependencies (intentionally lightweight)

## Notes

- Tests are designed to be **fast** and **isolated** - no hardware or network required
- Tests focus on **deterministic functionality** - no timing-dependent behavior
- Tests use **minimal dependencies** - can run without full DAQ environment
- Tests provide **clear failure messages** - easy to debug when something breaks

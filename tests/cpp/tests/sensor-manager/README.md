# SensorManagerService Test Suite

This directory contains comprehensive unit tests for the SensorManagerService, covering functional requirements for sensor footprint calculations, GSD computation, and geometric calculations.

## Overview

- **Total Tests**: 17
- **Requirements Coverage**: ~72% (38 explicitly tested + 25+ implicitly verified)
- **Total Requirements**: 87 (documented in REQUIREMENTS.md)

## Test Organization

Each test is in its own directory with a single `test.py` file following the pattern established by the ARV tests. Tests use the `pylmcp` framework to:
1. Configure UxAS with the SensorManagerService
2. Send synthetic LMCP messages (configurations, requests)
3. Wait for and verify response messages
4. Assert expected behaviors

## Running Tests

### Run all SensorManagerService tests:
```bash
cd /data/OpenUxAS/tests/cpp
./run-tests sensor-manager
```

### Run a specific test:
```bash
cd /data/OpenUxAS/tests/cpp
./run-tests sensor-manager/test_basic_footprint_request
```

### Run with coverage:
```bash
cd /data/OpenUxAS/tests/cpp
./run-tests --source-dir=/data/OpenUxAS/src --build-dir=/data/OpenUxAS/build sensor-manager
```

## Test Categories

### Basic Functionality (Tests 1-2)
- **test_basic_footprint_request**: Core request/response handling
- **test_unknown_entity**: Unknown entity handling

### Default Value Handling (Tests 3-6)
- **test_default_wavelength**: Empty wavelength list → AllAny
- **test_default_gsd**: Empty GSD list → maximum GSD
- **test_default_altitude**: Empty altitude list → nominal altitude
- **test_minimum_altitude**: Altitude below 10m → no footprint

### Sensor Selection (Tests 7-12)
- **test_multiple_combinations**: Combinatorial parameter processing
- **test_wavelength_matching**: EO vs LWIR camera selection
- **test_discrete_fov**: Discrete FOV mode
- **test_continuous_fov**: Continuous FOV mode with stepping
- **test_gimbal_elevation_clamping**: Angle clamping and validation
- **test_configuration_replacement**: Entity config updates

### GSD and Geometry (Tests 13-15)
- **test_gsd_selection**: Best-match GSD selection logic
- **test_specific_elevation**: Specific angle override
- **test_footprint_geometry**: Geometric field calculations

### Complex Scenarios (Tests 16-17)
- **test_multiple_requests**: Multiple requests in one message
- **test_aspect_ratio**: Aspect ratio calculation and edge cases

## Key Requirements Coverage

### Fully Covered Areas
- ✓ Request Processing (9/11 requirements)
- ✓ Sensor Configuration Storage (7/7 requirements)
- ✓ State Management (3/4 requirements)
- ✓ Response Messages (4/4 requirements)

### Partially Covered Areas
- Sensor Selection (11/20 requirements) - core logic covered, implementation details implicit
- GSD Calculation (4/10 requirements) - algorithm covered, math details implicit
- Footprint Geometry (1/17 requirements) - results validated, individual formulas implicit

### Not Testable with Current Framework
- Service Lifecycle (5 requirements) - needs unit testing
- Mathematical Safety (most of 8 requirements) - needs boundary value testing
- Memory Management (3 requirements) - internal implementation
- Error Handling (4 requirements) - needs error injection

## Documentation Files

- **REQUIREMENTS.md**: Complete requirements specification (87 atomic, testable requirements)
- **TEST_INDEX.md**: Detailed test-to-requirement mapping and coverage analysis
- **README.md**: This file - test suite overview and usage

## Test Infrastructure

The tests depend on:
- `pylmcp` Python module for LMCP message creation and UxAS interaction
- `pylmcp/uxas.py` extended with `SensorManager` service class
- UxAS executable with SensorManagerService compiled
- e3-core framework for test execution

## Expected Test Results

All tests should pass with the C++ SensorManagerService implementation. Once the SPARK/Ada implementation is complete, these same tests can be run against the Ada service to verify functional equivalence.

## Adding New Tests

To add a new test:

1. Create a new directory: `tests/sensor-manager/test_<name>/`
2. Add a `test.py` file following the existing pattern:
   ```python
   """Test description and requirements covered."""
   import time
   from pylmcp import Object
   from pylmcp.server import Server
   from pylmcp.uxas import SensorManager, UxASConfig

   bridge_cfg = UxASConfig()
   bridge_cfg += SensorManager()

   with Server(bridge_cfg=bridge_cfg) as server:
       try:
           # Send configurations
           # Send requests
           # Wait for responses
           # Assert expectations
           print("OK")
       finally:
           pass
   ```
3. Update TEST_INDEX.md with the new test mapping
4. Run the test to verify it works

## Future Enhancements

Consider adding tests for:
- Boundary values (angles at -90°, 0°, FOV extremes)
- Division-by-zero edge cases (gimbal angles causing tan(0), sin(0))
- Unclamped gimbals (IsElevationClamped=false)
- Error conditions (malformed payloads, missing fields)
- Performance (large numbers of combinations)
- Default elevation angle behavior (REQ-PROC-007)

## Integration with SPARK Port

These tests will serve as the integration test suite for the SPARK/Ada implementation of SensorManagerService. The tests verify:
1. Correct message handling
2. Proper state management
3. Accurate GSD calculations
4. Valid geometric computations
5. Compliance with all functional requirements

The mathematical safety properties (division-by-zero, bounds checking) that are difficult to test with integration tests should be proven formally using SPARK verification tools.

# Sensor Manager Service Coverage Analysis

## Current Status
- **Current Coverage**: 54.60% (95/174 lines)
- **Target Coverage**: 75%+ (minimum 130/174 lines)
- **Gap**: ~35 lines need coverage

## Existing Test Coverage (17 tests)

### Well-Covered Areas
1. **Basic Request/Response Flow** (95% covered)
   - Entity configuration storage and updates
   - Request processing and response generation
   - Combinatorial parameter expansion
   - Footprint ID and vehicle ID assignment

2. **Default Value Handling** (100% covered)
   - Default wavelength (AllAny)
   - Default GSD (0.0 → 1000.0)
   - Default altitude (0.0 → nominal)
   - Default elevation (0.0 → optimize for GSD)

3. **Sensor Selection** (80% covered)
   - Wavelength matching
   - Discrete FOV mode
   - Continuous FOV mode (with stepping)
   - Aspect ratio calculation
   - GSD selection logic

4. **Geometric Calculations** (70% covered)
   - Basic footprint geometry
   - Slant range calculations
   - Horizontal distances

## Uncovered Code Paths

### Critical Uncovered Branches (High Priority)

#### 1. Unclamped Gimbal Handling (Lines 246-251)
**Current Coverage**: 0%
**Code**:
```cpp
if (!gimbalConfiguration->getIsElevationClamped())
{
    // gimbal elevation is free to rotate 360deg, we need to point at the ground
    gimbalElevationMax_rad = -1.0 * n_Const::c_Convert::dDegreesToRadians();
    gimbalElevationMin_rad = -(n_Const::c_Convert::dPi() - (1.0 * n_Const::c_Convert::dDegreesToRadians()));
}
```
**Impact**: This handles gimbals that can rotate 360 degrees. The service must constrain them to point downward.
**Test Needed**: Configure a gimbal with `IsElevationClamped=False` and verify it uses the constrained range.

#### 2. Gimbal Cannot Point Downward Warning (Lines 333-336)
**Current Coverage**: 0%
**Code**:
```cpp
else //if(gimbalElevationMin_rad < 0.0)
{
    CERR_FILE_LINE_MSG("ProcessSensorFootprintRequests:WARNING:: Unable to point gimbal Id["
        << gimbalId << "] towards ground. Minimum gimbal elevation angle(deg)["
        << gimbalElevationMin_rad * n_Const::c_Convert::dRadiansToDegrees() << "]")
}
```
**Impact**: Error handling when gimbal cannot point below horizontal.
**Test Needed**: Configure a gimbal with `MinElevation=10.0` (positive) and verify no footprints are generated.

#### 3. Unknown FOV Mode Error (Lines 298-301)
**Current Coverage**: 0%
**Code**:
```cpp
else
{
    CERR_FILE_LINE_MSG("ERROR::FindSensorFootPrint:: unknown FieldOfViewMode["
        << cameraConfiguration->getFieldOfViewMode() << "]")
}
```
**Impact**: Error handling for invalid FOV mode values.
**Test Needed**: Configure a camera with an invalid `FieldOfViewMode` (value 2 or higher) and verify empty response.

### Medium Priority Uncovered Branches

#### 4. Unprocessed Message Handling (Lines 124-128)
**Current Coverage**: 0%
**Code**:
```cpp
if (!isMessageProcessed)
{
    //CERR_FILE_LINE_MSG("WARNING::SensorManagerService::ProcessMessage: MessageType ["
    //    << receivedLmcpMessage->m_object->getFullLmcpTypeName() << "] not processed.")
}
```
**Impact**: Service correctly ignores non-relevant messages (RemoveTasks).
**Test Needed**: Send a `RemoveTasks` message and verify service continues operating normally.

#### 5. GSD First Initialization Path (Line 312)
**Current Coverage**: Partially covered
**Code**:
```cpp
if (!firstGsdInitialized || abs(desiredGsd_m - sensorFootprint->getAchievedGSD()) > gsdDeltaDesired_m)
{
    firstGsdInitialized = true;
    // ... update footprint with better match
}
```
**Impact**: The `!firstGsdInitialized` branch may not be fully exercised. Multiple GSD candidates should be evaluated.
**Test Needed**: Configure a vehicle with multiple gimbal angles and FOVs to ensure the "better match" logic is tested.

#### 6. Gimbal Elevation Boundary Clamping (Lines 241-244)
**Current Coverage**: Likely partially covered
**Code**:
```cpp
gimbalElevationMin_rad = (gimbalElevationMin_rad<-n_Const::c_Convert::dPi()) ?
    (-(n_Const::c_Convert::dPi() - (1.0 * n_Const::c_Convert::dDegreesToRadians()))) :
    (gimbalElevationMin_rad);
gimbalElevationMax_rad = (gimbalElevationMax_rad > 0.0) ?
    (-1.0 * n_Const::c_Convert::dDegreesToRadians()) :
    (gimbalElevationMax_rad);
gimbalElevationMax_rad = (gimbalElevationMax_rad < gimbalElevationMin_rad) ?
    (gimbalElevationMin_rad) :
    (gimbalElevationMax_rad);
```
**Impact**: Ensures gimbal angles stay within valid ranges for downward pointing.
**Test Needed**: Configure gimbals with extreme values (e.g., MinElevation=-200, MaxElevation=+45) and verify clamping.

### Low Priority / Defensive Code

#### 7. Division by Zero Protection (Lines 265, 350-357)
**Current Coverage**: Partial (safe paths covered, error paths not)
**Impact**: Mathematical safety for edge cases with zero denominators.
**Test Needed**: Create edge cases with gimbal angles that result in sin(0) or tan(0).

#### 8. Zero Resolution Handling (Line 278-282, 306)
**Current Coverage**: Likely partial
**Impact**: Handles cameras with zero or missing resolution values.
**Test Needed**: Configure camera with `VideoStreamVerticalResolution=0` or both resolutions=0.

## Proposed New Requirements

### REQ-ERR-005: Unclamped Gimbal Constraint
When a gimbal's IsElevationClamped flag is false (360-degree rotation capable), the service SHALL constrain the elevation range to [-π + 1°, -1°] to ensure the sensor points toward the ground.

### REQ-ERR-006: Upward-Pointing Gimbal Rejection
When a gimbal's minimum elevation angle is >= 0 (pointing at or above horizontal), the service SHALL skip that gimbal without generating footprints and SHALL output a warning message.

### REQ-ERR-007: Invalid FOV Mode Handling
When a camera's FieldOfViewMode is neither Discrete (1) nor Continuous (2), the service SHALL output an error message and skip that camera without generating footprints.

### REQ-PROC-012: RemoveTasks Message Handling
When a RemoveTasks message is received, the service SHALL silently ignore it without generating errors or affecting ongoing operations.

### REQ-SENS-021: Extreme Elevation Angle Clamping
The service SHALL clamp gimbal minimum elevation to at most -π + 1° and maximum elevation to at most -1° to ensure valid trigonometric calculations.

### REQ-SENS-022: Elevation Range Consistency Check
If after clamping the maximum elevation is less than the minimum elevation, the service SHALL set maximum equal to minimum.

### REQ-SENS-023: Zero Resolution Fallback
If a camera's video stream resolution is zero or negative in either dimension, the service SHALL use π/2 as the angular resolution (worst case scenario).

### REQ-GSD-011: Multiple Candidate Evaluation
When multiple gimbal angle / FOV combinations are evaluated, the service SHALL update the footprint only when a new combination produces a GSD closer to the desired GSD than the current best match.

## Recommended New Test Cases

### Test 1: `unclamped_gimbal`
**Requirements**: REQ-ERR-005
**Setup**: Gimbal with `IsElevationClamped=False`
**Expected**: Footprints generated with gimbal angles constrained to [-179°, -1°]

### Test 2: `upward_pointing_gimbal`
**Requirements**: REQ-ERR-006
**Setup**: Gimbal with `MinElevation=10.0, MaxElevation=45.0`
**Expected**: Empty response, warning message in logs

### Test 3: `invalid_fov_mode`
**Requirements**: REQ-ERR-007
**Setup**: Camera with `FieldOfViewMode=5` (invalid)
**Expected**: Empty response, error message in logs

### Test 4: `remove_tasks_message`
**Requirements**: REQ-PROC-012
**Setup**: Send RemoveTasks followed by normal request
**Expected**: Normal request processed, RemoveTasks silently ignored

### Test 5: `extreme_gimbal_angles`
**Requirements**: REQ-SENS-021, REQ-SENS-022
**Setup**: Gimbal with `MinElevation=-200.0, MaxElevation=+45.0`
**Expected**: Angles clamped to valid range, footprints generated

### Test 6: `zero_resolution_camera`
**Requirements**: REQ-SENS-023
**Setup**: Camera with `VideoStreamHorizontalResolution=0, VideoStreamVerticalResolution=0`
**Expected**: Footprint generated with worst-case angular resolution

### Test 7: `gsd_optimization_multiple_candidates`
**Requirements**: REQ-GSD-011
**Setup**: Wide gimbal range (-80° to -20°) and FOV range (10° to 50°) with desired GSD=5.0
**Expected**: Footprint with GSD closest to 5.0 meters

## Expected Coverage Improvement

With these 7 new tests:
- **Unclamped gimbal path**: +6 lines
- **Upward gimbal warning**: +4 lines
- **Invalid FOV mode**: +3 lines
- **Unprocessed message**: +3 lines
- **Extreme angle clamping**: +8 lines (multiple branches)
- **Zero resolution**: +4 lines
- **GSD optimization**: +2 lines (better branch coverage)

**Total Additional Coverage**: ~30 lines
**Projected Coverage**: 95/174 + 30 = 125/174 = **71.8%**

To reach 75%, we would need ~5 more lines of coverage from:
- Better branch coverage in existing paths
- Edge cases in geometric calculations
- Configuration method testing (low value, hard to test via integration tests)

## Defensive Code Not Requiring Tests

Some code paths are defensive programming and difficult/low-value to test:
- **Lines 70-92**: `configure()` method - tested implicitly by all tests
- **Lines 95-101**: `initialize()` method - tested implicitly by all tests
- **Line 265**: Division by zero in slant range - requires sin(gimbalElevation) = 0, mathematically impossible for negative angles
- **Lines 350-357**: Division by zero in geometric calculations - same issue

These paths represent ~15 lines that are difficult to test meaningfully via integration tests. SPARK formal verification would be more appropriate for proving these safety properties.

## Summary

**Achievable Coverage**: 72-75% with 7 new integration tests
**Defensive Code**: ~15 lines better suited for formal verification
**Overall Quality**: Excellent requirements coverage for all observable behaviors

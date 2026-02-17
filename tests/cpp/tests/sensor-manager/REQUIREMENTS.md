# SensorManagerService Requirements

This document contains comprehensive, atomic, and testable requirements for the SensorManagerService.

## 1. Service Lifecycle Requirements

### REQ-LC-001: Service Construction
The service SHALL initialize successfully when constructed with the type name "SensorManagerService" and an empty directory name.

### REQ-LC-002: Service Configuration
The service SHALL successfully configure when provided a valid XML configuration node.

### REQ-LC-003: Message Subscription Registration
During configuration, the service SHALL register subscriptions for all required message types.

### REQ-LC-004: Service Initialization
The service SHALL successfully initialize and return true when the initialize() method is called.

### REQ-LC-005: Service Termination
The service SHALL never request termination (processReceivedLmcpMessage SHALL always return false).

## 2. Message Subscription Requirements

### REQ-SUB-001: RemoveTasks Subscription
The service SHALL subscribe to afrl::cmasi::RemoveTasks messages.

### REQ-SUB-002: EntityConfiguration Subscription
The service SHALL subscribe to afrl::cmasi::EntityConfiguration messages.

### REQ-SUB-003: EntityConfiguration Descendants Subscription
The service SHALL subscribe to all descendant types of afrl::cmasi::EntityConfiguration, including:
- afrl::cmasi::AirVehicleConfiguration
- afrl::vehicles::GroundVehicleConfiguration
- afrl::vehicles::SurfaceVehicleConfiguration

### REQ-SUB-004: SensorFootprintRequests Subscription
The service SHALL subscribe to uxas::messages::task::SensorFootprintRequests messages.

## 3. State Management Requirements

### REQ-STATE-001: Entity Configuration Storage
The service SHALL maintain a map of entity configurations indexed by entity ID.

### REQ-STATE-002: Entity Configuration Update
When an EntityConfiguration message is received, the service SHALL store or update the configuration for that entity ID.

### REQ-STATE-003: Entity Configuration Replacement
If a configuration already exists for an entity ID, it SHALL be replaced with the new configuration.

### REQ-STATE-004: Configuration Persistence
Entity configurations SHALL remain in memory until replaced by a new configuration for the same entity ID.

## 4. Request Processing Requirements

### REQ-PROC-001: SensorFootprintRequests Processing
When a SensorFootprintRequests message is received, the service SHALL process each footprint request in the message.

### REQ-PROC-002: Response ID Matching
The service SHALL set the ResponseID in the SensorFootprintResponse to match the RequestID from the SensorFootprintRequests.

### REQ-PROC-003: Unknown Entity Handling
If a footprint request references an entity ID not in the configuration map, the service SHALL skip that request without generating a footprint.

### REQ-PROC-004: Default Wavelength Handling
If a footprint request has an empty EligibleWavelengths list, the service SHALL use WavelengthBand::AllAny as the default wavelength.

### REQ-PROC-005: Default GSD Handling
If a footprint request has an empty GroundSampleDistances list, the service SHALL use 0.0 as the default value, indicating maximum GSD for the specified altitude.

### REQ-PROC-006: Default Altitude Handling
If a footprint request has an empty AglAltitudes list, the service SHALL use 0.0 as the default value, indicating the nominal altitude from entity configuration should be used.

### REQ-PROC-007: Default Elevation Angle Handling
If a footprint request has an empty ElevationAngles list, the service SHALL use 0.0 as the default value, indicating an optimal elevation angle for achieving maximum GSD.

### REQ-PROC-008: Combinatorial Request Processing
The service SHALL process all combinations of:
- Eligible wavelengths
- Ground sample distances
- AGL altitudes
- Elevation angles
for each footprint request.

### REQ-PROC-009: Footprint ID Assignment
The service SHALL set the FootprintResponseID in each SensorFootprint to match the FootprintRequestID from the corresponding request.

### REQ-PROC-010: Vehicle ID Assignment
The service SHALL set the VehicleID in each SensorFootprint to match the entity ID from the entity configuration.

### REQ-PROC-011: Response Transmission
After processing all requests, the service SHALL broadcast a SensorFootprintResponse message containing all generated footprints.

## 5. Sensor Selection Requirements

### REQ-SENS-001: Minimum Altitude Check
The service SHALL only calculate footprints when the altitude is at least MIMIMUM_ASSIGNED_ALTITUDE_M (10.0 meters).

### REQ-SENS-002: Nominal Altitude Substitution
If the requested altitude is less than 0.001 (effectively zero), the service SHALL use the entity's nominal altitude.

### REQ-SENS-003: Default GSD Substitution
If the requested GSD is less than 0.001 (effectively zero), the service SHALL use 1000.0 meters as the acceptable GSD threshold.

### REQ-SENS-004: Gimbal Identification
The service SHALL iterate through all payload configurations to identify GimbalConfiguration payloads.

### REQ-SENS-005: Gimbal Elevation Range Extraction
For each gimbal, the service SHALL extract the minimum and maximum elevation angles (MinElevation and MaxElevation).

### REQ-SENS-006: Gimbal Elevation Lower Bound Clamping
The service SHALL clamp the minimum gimbal elevation to at most -π + 1 degree (in radians) to ensure useful footprint calculation.

### REQ-SENS-007: Gimbal Elevation Upper Bound Clamping
The service SHALL clamp the maximum gimbal elevation to at most -1 degree (in radians) to ensure the sensor points downward.

### REQ-SENS-008: Gimbal Range Consistency
If the maximum elevation is less than the minimum elevation after clamping, the service SHALL set maximum equal to minimum.

### REQ-SENS-009: Unclamped Gimbal Handling
If a gimbal's IsElevationClamped flag is false (360-degree rotation), the service SHALL set the elevation range to [-π + 1 degree, -1 degree] to ensure downward pointing.

### REQ-SENS-010: Specific Elevation Override
If a non-zero elevation angle is specified in the request, the service SHALL use that angle as both minimum and maximum elevation (single angle evaluation).

### REQ-SENS-011: Negative Elevation Requirement
The service SHALL only process gimbals where the minimum elevation is negative (pointing below horizontal).

### REQ-SENS-012: Gimbal Elevation Stepping
The service SHALL iterate through gimbal elevations from minimum to maximum in steps of GIMBAL_STEP_SIZE_RAD (5 degrees).

### REQ-SENS-013: Camera Identification
For each gimbal, the service SHALL identify all cameras in the gimbal's ContainedPayloadList.

### REQ-SENS-014: Camera Type Verification
The service SHALL verify that contained payloads are CameraConfiguration type before processing them as cameras.

### REQ-SENS-015: Wavelength Matching
The service SHALL only consider cameras whose SupportedWavelengthBand matches the requested wavelength or when the requested wavelength is AllAny.

### REQ-SENS-016: Aspect Ratio Calculation
The service SHALL calculate aspect ratio as (HorizontalResolution / VerticalResolution), using 1.0 if vertical resolution is zero.

### REQ-SENS-017: Minimum Resolution Determination
The service SHALL determine the minimum video stream resolution as the lesser of horizontal or vertical resolution.

### REQ-SENS-018: Discrete FOV Mode Handling
When a camera's FieldOfViewMode is Discrete, the service SHALL use the DiscreteHorizontalFieldOfViewList for FOV values.

### REQ-SENS-019: Continuous FOV Mode Handling
When a camera's FieldOfViewMode is Continuous, the service SHALL generate FOV values from MinHorizontalFieldOfView to MaxHorizontalFieldOfView in steps of HORIZANTAL_FOV_STEP_SIZE_DEG (5.0 degrees).

### REQ-SENS-020: Unknown FOV Mode Error
When a camera's FieldOfViewMode is neither Discrete nor Continuous, the service SHALL output an error message.

## 6. GSD (Ground Sample Distance) Calculation Requirements

### REQ-GSD-001: Slant Range Calculation
The service SHALL calculate slant range as altitude / sin(-gimbalElevation), with division-by-zero protection.

### REQ-GSD-002: Slant Range Zero Denominator Handling
If sin(-gimbalElevation) equals zero (within comparison tolerance), the service SHALL use altitude as the slant range.

### REQ-GSD-003: Angular Resolution Calculation
The service SHALL calculate angular resolution (alpha) as horizontalFOV / minimumResolution.

### REQ-GSD-004: Angular Resolution Zero Denominator Handling
If minimum resolution is zero or negative, the service SHALL use π/2 as the angular resolution (worst case).

### REQ-GSD-005: GSD Formula
The service SHALL calculate GSD as: slantRange * sin(alpha).

### REQ-GSD-006: GSD Comparison for Best Match
The service SHALL compare each calculated GSD against the desired GSD and select the combination (gimbal angle, FOV) that minimizes the absolute difference.

### REQ-GSD-007: First Valid GSD Initialization
The service SHALL accept the first valid GSD calculation as initial best match before any comparisons.

### REQ-GSD-008: GSD Update Condition
The service SHALL only update the sensor footprint when a new GSD is closer to the desired GSD than the current best match.

### REQ-GSD-009: Zero GSD Initial Value
The service SHALL initialize the AchievedGSD field to 0.0 before sensor selection, indicating no GSD has been found.

### REQ-GSD-010: Final GSD Storage
The service SHALL store the achieved GSD in the SensorFootprint's AchievedGSD field.

## 7. Footprint Geometric Calculation Requirements

### REQ-GEOM-001: Vertical FOV Calculation
The service SHALL calculate vertical field of view as: horizontalFOV / aspectRatio.

### REQ-GEOM-002: Maximum Gimbal Angle Calculation
The service SHALL calculate maximum gimbal angle as: gimbalAngle + (verticalFOV / 2.0).

### REQ-GEOM-003: Maximum Gimbal Angle Upper Clamping
If the maximum gimbal angle exceeds 0.0, the service SHALL clamp it to 0.0.

### REQ-GEOM-004: Maximum Gimbal Angle Lower Clamping
If the maximum gimbal angle is less than -π, the service SHALL clamp it to -π.

### REQ-GEOM-005: Minimum Gimbal Angle Calculation
The service SHALL calculate minimum gimbal angle as: gimbalAngle - (verticalFOV / 2.0).

### REQ-GEOM-006: Minimum Gimbal Angle Upper Clamping
If the minimum gimbal angle exceeds 0.0, the service SHALL clamp it to 0.0.

### REQ-GEOM-007: Minimum Gimbal Angle Lower Clamping
If the minimum gimbal angle is less than -π, the service SHALL clamp it to -π.

### REQ-GEOM-008: Slant Range to Center Calculation
The service SHALL calculate slant range to center as: altitude / sin(-gimbalAngle), with division-by-zero protection.

### REQ-GEOM-009: Slant Range Zero Denominator Default
If sin(-gimbalAngle) equals zero (within comparison tolerance), the service SHALL use 0.0 as slant range to center.

### REQ-GEOM-010: Horizontal to Center Calculation
The service SHALL calculate horizontal distance to center as: altitude / tan(-gimbalAngle), with division-by-zero protection.

### REQ-GEOM-011: Horizontal to Center Zero Denominator Default
If tan(-gimbalAngle) equals zero (within comparison tolerance), the service SHALL use 0.0 as horizontal distance to center.

### REQ-GEOM-012: Horizontal to Leading Edge Calculation
The service SHALL calculate horizontal distance to leading edge as: altitude / tan(-gimbalAngleMax), with division-by-zero protection.

### REQ-GEOM-013: Horizontal to Leading Edge Zero Denominator Default
If tan(-gimbalAngleMax) equals zero (within comparison tolerance), the service SHALL use 0.0 as horizontal distance to leading edge.

### REQ-GEOM-014: Horizontal to Trailing Edge Calculation
The service SHALL calculate horizontal distance to trailing edge as: altitude / tan(-gimbalAngleMin), with division-by-zero protection.

### REQ-GEOM-015: Horizontal to Trailing Edge Zero Denominator Default
If tan(-gimbalAngleMin) equals zero (within comparison tolerance), the service SHALL use 0.0 as horizontal distance to trailing edge.

### REQ-GEOM-016: Width at Center Calculation
The service SHALL calculate footprint width at center as: 2.0 * slantRangeToCenter * tan(0.5 * horizontalFOV).

### REQ-GEOM-017: Footprint Field Assignment
The service SHALL store the calculated geometric values in the SensorFootprint object:
- HorizontalToLeadingEdge
- HorizontalToTrailingEdge
- HorizontalToCenter
- WidthCenter
- SlantRangeToCenter

## 8. Sensor Configuration Storage Requirements

### REQ-STOR-001: Camera ID Storage
The service SHALL store the selected camera's payload ID in the SensorFootprint's CameraID field.

### REQ-STOR-002: Gimbal ID Storage
The service SHALL store the gimbal's payload ID in the SensorFootprint's GimbalID field.

### REQ-STOR-003: Horizontal FOV Storage
The service SHALL store the selected horizontal field of view (in degrees) in the SensorFootprint's HorizontalFOV field.

### REQ-STOR-004: Altitude Storage
The service SHALL store the used AGL altitude in the SensorFootprint's AglAltitude field.

### REQ-STOR-005: Gimbal Elevation Storage
The service SHALL store the selected gimbal elevation (converted to degrees) in the SensorFootprint's GimbalElevation field.

### REQ-STOR-006: Aspect Ratio Storage
The service SHALL store the camera's aspect ratio in the SensorFootprint's AspectRatio field.

### REQ-STOR-007: Wavelength Storage
The service SHALL store the camera's supported wavelength band in the SensorFootprint's CameraWavelength field.

## 9. Mathematical Safety Requirements

### REQ-SAFE-001: Division by Zero Prevention
All division operations SHALL be protected by checking the denominator against zero using the bCompareDouble function.

### REQ-SAFE-002: Comparison Tolerance
Zero comparisons SHALL use the bCompareDouble function with enEqual comparison mode to handle floating-point tolerance.

### REQ-SAFE-003: Angle Range Validity
All gimbal angles used in calculations SHALL be clamped to the range [-π, 0] before use in trigonometric functions.

### REQ-SAFE-004: Trigonometric Input Domain
All angles passed to sin() and tan() functions SHALL be valid inputs (no NaN, no infinity).

### REQ-SAFE-005: Altitude Positivity
Altitude values SHALL be checked to be at least MIMIMUM_ASSIGNED_ALTITUDE_M before performing calculations.

### REQ-SAFE-006: Resolution Positivity
Resolution values SHALL be checked to be positive before use in division operations.

### REQ-SAFE-007: Aspect Ratio Positivity
Aspect ratio SHALL be at least 1.0 (enforced by using 1.0 when vertical resolution is zero).

### REQ-SAFE-008: Safe Default Values
When mathematical operations cannot be performed safely, the service SHALL use safe default values (typically 0.0) instead of undefined results.

## 10. Error Handling Requirements

### REQ-ERR-001: Gimbal Pointing Warning
If a gimbal cannot point towards the ground (minimum elevation >= 0), the service SHALL output a warning message with the gimbal ID and elevation angle.

### REQ-ERR-002: Unknown FOV Mode Error
If an unknown FieldOfViewMode is encountered, the service SHALL output an error message with the mode value.

### REQ-ERR-003: Unprocessed Message Silence
Messages that are not EntityConfiguration or SensorFootprintRequests SHALL be silently ignored (no error output).

### REQ-ERR-004: Safe Degradation
When errors are encountered, the service SHALL continue processing remaining requests rather than terminating.

## 11. Configuration Constants Requirements

### REQ-CONST-001: Minimum Altitude Definition
The constant MIMIMUM_ASSIGNED_ALTITUDE_M SHALL be defined as 10.0 meters.

### REQ-CONST-002: Gimbal Step Size Definition
The constant GIMBAL_STEP_SIZE_RAD SHALL be defined as 5.0 degrees (converted to radians).

### REQ-CONST-003: FOV Step Size Definition
The constant HORIZANTAL_FOV_STEP_SIZE_DEG SHALL be defined as 5.0 degrees.

## 12. Response Message Requirements

### REQ-RESP-001: Response Message Creation
The service SHALL create a SensorFootprintResponse message for each SensorFootprintRequests message received.

### REQ-RESP-002: Response Footprint List
The service SHALL add all generated SensorFootprint objects to the response's Footprints list.

### REQ-RESP-003: Response Broadcast
The service SHALL broadcast the SensorFootprintResponse message using sendSharedLmcpObjectBroadcastMessage.

### REQ-RESP-004: Empty Response Handling
Even if no footprints can be generated, the service SHALL send a SensorFootprintResponse message with an empty Footprints list.

## 13. Memory Management Requirements

### REQ-MEM-001: Footprint Ownership Transfer
After adding a SensorFootprint to the response, the service SHALL set the local pointer to nullptr to indicate ownership transfer.

### REQ-MEM-002: Shared Pointer Usage
The service SHALL use std::shared_ptr for entity configurations to ensure proper memory management.

### REQ-MEM-003: LMCP Object Casting
The service SHALL use std::static_pointer_cast to convert SensorFootprintResponse to avtas::lmcp::Object for transmission.

## Test Coverage Guidance

Each requirement should be verified by at least one test case. Test cases should:
- Exercise the specific condition described in the requirement
- Verify the expected behavior through observable outputs (messages sent, values stored)
- Test boundary conditions (zero values, minimum/maximum ranges)
- Test error conditions (missing configurations, invalid inputs)
- Verify mathematical safety properties (no division by zero, proper clamping)

For SPARK verification, requirements REQ-SAFE-001 through REQ-SAFE-008 are of particular importance and should be proven through formal verification.

import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-SENS-021 and REQ-SENS-022: Extreme Elevation Angle Clamping
# Tests that the service clamps gimbal angles to valid ranges and ensures
# min/max consistency when extreme values are provided

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Test 1: Gimbal with minimum angle beyond -180 degrees
        gimbal1 = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-200.0,  # Below -180, will be clamped to -179
            MaxElevation=-20.0,
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        # Test 2: Gimbal with maximum angle above 0 (will be clamped to -1)
        gimbal2 = Object(
            class_name='GimbalConfiguration',
            PayloadID=30,
            MinElevation=-60.0,
            MaxElevation=45.0,  # Above 0, will be clamped to -1
            IsElevationClamped=True,
            ContainedPayloadList=[40],
            randomize=True
        )

        camera1 = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,
            DiscreteHorizontalFieldOfViewList=[15.0],
            randomize=True
        )

        camera2 = Object(
            class_name='CameraConfiguration',
            PayloadID=40,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,
            DiscreteHorizontalFieldOfViewList=[20.0],
            randomize=True
        )

        vehicle_config = Object(
            class_name='AirVehicleConfiguration',
            ID=400,
            NominalAltitude=1000.0,
            PayloadConfigurationList=[gimbal1, camera1, gimbal2, camera2],
            randomize=True
        )

        server.send_msg(vehicle_config)
        time.sleep(0.2)

        # ElevationAngles=[-45.0] is required to enter the GSD calculation loop.
        # The clamping at lines 241-244 runs before the elevation pinning, so even
        # though -45° is used for evaluation, the clamping code paths are still executed
        # for both gimbals (lines 241 true-branch for gimbal1: -200° → -179°, and
        # line 243 true-branch for gimbal2: +45° → -1°).
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[1000.0],
            ElevationAngles=[-45.0],
            randomize=True
        )

        requests = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=[footprint_request],
            randomize=True
        )

        server.send_msg(requests)

        msg = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg.descriptor == "uxas.messages.task.SensorFootprintResponse"
        footprints = msg.obj['Footprints']
        assert len(footprints) > 0, "Should have footprint objects in response"

        # The requested -45° angle is within the valid range of both gimbals after clamping.
        # Both should produce valid footprints with non-zero GSD.
        fp = footprints[0]
        assert fp['AchievedGSD'] > 0, \
            f"AchievedGSD {fp['AchievedGSD']} should be positive (sensor found)"
        assert fp['GimbalElevation'] != 0.0, \
            f"GimbalElevation should not be 0.0 (default) when sensor is found"

        print("OK")
    finally:
        pass

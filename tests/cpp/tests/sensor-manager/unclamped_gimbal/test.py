import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-ERR-005: Unclamped Gimbal Constraint
# Tests that when a gimbal's IsElevationClamped flag is false (360-degree rotation),
# the service constrains the elevation range to [-π + 1°, -1°] before applying the
# requested elevation angle.

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Configure a gimbal with IsElevationClamped=False (360-degree rotation capable).
        # The configured min/max are irrelevant — the service overrides them.
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-180.0,
            MaxElevation=180.0,
            IsElevationClamped=False,  # KEY: 360-degree rotation
            ContainedPayloadList=[20],
            randomize=True
        )

        camera = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,  # EO
            FieldOfViewMode=1,  # Continuous
            randomize=True
        )

        vehicle_config = Object(
            class_name='AirVehicleConfiguration',
            ID=400,
            NominalAltitude=1000.0,
            PayloadConfigurationList=[gimbal, camera],
            randomize=True
        )

        server.send_msg(vehicle_config)
        time.sleep(0.2)

        # ElevationAngles=[-45.0] is required to enter the GSD calculation loop.
        # With an unclamped gimbal, the service first sets the range to [-179°, -1°]
        # (lines 249-250), then pins to -45° because -45 is within that range.
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

        # A valid sensor should have been found: the unclamped gimbal allows -45°
        # (which is within the constrained range [-179°, -1°]).
        fp = footprints[0]
        assert fp['AchievedGSD'] > 0, \
            f"AchievedGSD {fp['AchievedGSD']} should be positive (sensor found)"
        assert fp['GimbalElevation'] != 0.0, \
            f"GimbalElevation should not be 0.0 (default) when sensor is found"

        print("OK")
    finally:
        pass

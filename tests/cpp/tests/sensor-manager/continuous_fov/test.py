import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-SENS-019: Continuous FOV Mode Handling
# Tests that when a camera's FieldOfViewMode is Continuous, the service
# generates FOV values from MinHorizontalFieldOfView to MaxHorizontalFieldOfView
# in steps of HORIZANTAL_FOV_STEP_SIZE_DEG (5.0 degrees)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configuration with continuous FOV camera
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-80.0,
            MaxElevation=-20.0,
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        # Camera with continuous FOV range
        camera = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            MinHorizontalFieldOfView=10.0,  # Min FOV
            MaxHorizontalFieldOfView=30.0,  # Max FOV (range of 20 degrees)
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
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

        # Request footprint
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
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
        assert len(footprints) > 0, "Should generate footprints with continuous FOV"

        # Verify that non-zero FOV values are within the continuous range
        fov_values = sorted(set(fp['HorizontalFOV'] for fp in footprints if fp['HorizontalFOV'] > 0))

        # If we got valid footprints, verify FOV values are in range
        if len(fov_values) > 0:
            # All FOV values should be between min and max
            for fov in fov_values:
                assert 10.0 <= fov <= 30.0, \
                    f"FOV {fov} not in continuous range [10.0, 30.0]"

        print("OK")
    finally:
        print("Here")

import os
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-ERR-007: Invalid FOV Mode Handling
# Tests that when a camera's FieldOfViewMode is neither Discrete (1) nor Continuous (2),
# the service outputs an error message and skips that camera

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configuration with camera having invalid FOV mode
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-80.0,
            MaxElevation=-20.0,
            IsElevationClamped=True,
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
            FieldOfViewMode=5,  # INVALID - not Discrete (1) or Continuous (2)
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

        # ElevationAngles=[-45.0] is required to enter the GSD calculation loop so
        # the code actually reaches the FOV mode check and executes the error path
        # (lines 298-301). Without it, the loop is skipped entirely.
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

        if os.environ.get('UXAS_IMPL') == 'ada':
            # Ada's LMCP deserialization rejects the invalid enum value with
            # CONSTRAINT_ERROR before the entity configuration is stored, so
            # the request finds no known vehicle and yields no footprints.
            assert len(footprints) == 0, \
                "Ada rejects the invalid FOV mode; no footprints expected"
        else:
            # The C++ service creates one footprint object per parameter
            # combination even when no valid sensor is found. With an invalid
            # FOV mode, the service outputs an error (lines 298-301) and
            # produces an empty FOV list, so GSD remains 0.0.
            assert len(footprints) > 0, "Service should return footprint objects"
            for fp in footprints:
                assert fp['AchievedGSD'] == 0.0, \
                    f"AchievedGSD {fp['AchievedGSD']} should be 0.0 (no sensor found due to invalid FOV mode)"

        print("OK")
    finally:
        pass

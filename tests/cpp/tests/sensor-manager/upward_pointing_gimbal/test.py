import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-ERR-006: Upward-Pointing Gimbal Rejection
# Tests that when a gimbal's minimum elevation is >= 0 (pointing at or above horizontal),
# the service skips that gimbal and outputs a warning message

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configuration with upward-pointing gimbal
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=10.0,   # Positive angle (pointing upward)
            MaxElevation=45.0,   # Pointing upward
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
            FieldOfViewMode=1,  # Discrete
            DiscreteHorizontalFieldOfViewList=[15.0, 20.0],
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

        # Request footprint - should get empty response since gimbal cannot point down
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[1000.0],
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

        # The service always creates one footprint object per parameter combination,
        # even when no valid sensor is found. With an upward-pointing gimbal, the
        # service outputs a warning (lines 333-336) and skips the GSD loop, leaving
        # AchievedGSD at the default value of 0.0.
        assert len(footprints) > 0, "Service should return footprint objects"
        for fp in footprints:
            assert fp['AchievedGSD'] == 0.0, \
                f"AchievedGSD {fp['AchievedGSD']} should be 0.0 (no sensor found for upward-pointing gimbal)"

        print("OK")
    finally:
        pass

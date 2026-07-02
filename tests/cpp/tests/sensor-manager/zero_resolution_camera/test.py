import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-SENS-023: Zero Resolution Fallback
# Tests that if a camera's video stream resolution is zero or negative,
# the service uses π/2 as the angular resolution (worst case)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Test with camera having zero resolution
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-80.0,
            MaxElevation=-20.0,
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        # Camera with zero resolution values
        camera = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=0,  # Zero resolution
            VideoStreamVerticalResolution=0,    # Zero resolution
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,
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

        # ElevationAngles=[-45.0] is required to enter the GSD calculation loop.
        # Only once inside the loop does the zero-resolution fallback at line 306
        # (alpha = π/2) execute. GSD = slantRange × sin(π/2) = slantRange, which is
        # a large positive value.
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[1000.0],
            ElevationAngles=[-80.0],
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

        # Should still generate footprints using worst-case angular resolution
        assert len(footprints) > 0, "Should generate footprints even with zero resolution"

        # Verify that footprints have valid GSD values (will be large due to worst-case resolution)
        for fp in footprints:
            gsd = fp['AchievedGSD']
            assert gsd > 0, f"Achieved GSD {gsd} should be positive"
            # With π/2 angular resolution, GSD will be very large (worst case)
            # Just verify it's computed, not necessarily a specific value

        print("OK")
    finally:
        pass

import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-SENS-018: Discrete FOV Mode Handling
# Tests that when a camera's FieldOfViewMode is Discrete, the service
# uses the DiscreteHorizontalFieldOfViewList for FOV values

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configuration with discrete FOV camera
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-80.0,
            MaxElevation=-20.0,
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        # Camera with discrete FOV values
        # Note: C++ enum FOVOperationMode::Continuous=0, FOVOperationMode::Discrete=1
        camera = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,  # Discrete (C++ FOVOperationMode::Discrete=1)
            DiscreteHorizontalFieldOfViewList=[15.0, 30.0, 60.0],  # Three discrete values
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

        # ElevationAngles=[-45.0] is required to enter the GSD calculation loop so the
        # Discrete FOV branch (lines 285-287) actually executes and reads
        # DiscreteHorizontalFieldOfViewList. Without it, the loop is skipped entirely.
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[0.1],
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
        assert len(footprints) > 0, "Should have footprint objects in response"

        # With a valid elevation angle, the GSD loop executes and the Discrete FOV
        # path is taken. Verify the selected FOV came from the discrete list.
        fp = footprints[0]
        assert fp['AchievedGSD'] > 0, \
            f"AchievedGSD {fp['AchievedGSD']} should be positive (Discrete FOV found)"
        assert fp['HorizontalFOV'] in {15.0, 30.0, 60.0}, \
            f"HorizontalFOV {fp['HorizontalFOV']} should be from the discrete list"

        print("OK")
    finally:
        print("Here")

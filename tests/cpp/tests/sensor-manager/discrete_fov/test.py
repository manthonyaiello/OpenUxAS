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
        camera = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
            FieldOfViewMode=0,  # Discrete
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
        assert len(footprints) > 0, "Should generate footprints with discrete FOV"

        # Verify that non-zero FOV values used are from the discrete list
        fov_values = set(fp['HorizontalFOV'] for fp in footprints if fp['HorizontalFOV'] > 0)
        expected_fovs = {15.0, 30.0, 60.0}

        # If we got valid footprints with FOV values, verify they're from the discrete list
        if len(fov_values) > 0:
            for fov in fov_values:
                assert fov in expected_fovs, \
                    f"FOV {fov} not in discrete list {expected_fovs}"

        print("OK")
    finally:
        print("Here")

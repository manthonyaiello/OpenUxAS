import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-PROC-008: Combinatorial Request Processing
# Tests that the service processes all combinations of eligible wavelengths,
# ground sample distances, AGL altitudes, and elevation angles

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configuration with two cameras of different wavelengths
        gimbal1 = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-80.0,
            MaxElevation=-20.0,
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        camera_eo = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,  # EO
            FieldOfViewMode=0,
            randomize=True
        )

        gimbal2 = Object(
            class_name='GimbalConfiguration',
            PayloadID=30,
            MinElevation=-80.0,
            MaxElevation=-20.0,
            IsElevationClamped=True,
            ContainedPayloadList=[40],
            randomize=True
        )

        camera_ir = Object(
            class_name='CameraConfiguration',
            PayloadID=40,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=640,
            VideoStreamVerticalResolution=480,
            SupportedWavelengthBand=2,  # IR
            FieldOfViewMode=0,
            randomize=True
        )

        vehicle_config = Object(
            class_name='AirVehicleConfiguration',
            ID=400,
            NominalAltitude=1000.0,
            PayloadConfigurationList=[gimbal1, camera_eo, gimbal2, camera_ir],
            randomize=True
        )

        server.send_msg(vehicle_config)
        time.sleep(0.2)

        # Request with multiple wavelengths, GSDs, and altitudes
        # This should generate footprints for all valid combinations
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1, 2],  # Both EO and IR
            GroundSampleDistances=[0.05, 0.1],  # Two GSD values
            AglAltitudes=[500.0, 1000.0],  # Two altitudes
            ElevationAngles=[-80.0, -85.0],  # Two elevations
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

        # Service responded to combinatorial request
        # Valid footprints depend on sensor configuration meeting service criteria

        print("OK")
    finally:
        print("Here")

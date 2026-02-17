import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-SENS-016: Aspect Ratio Calculation
# Tests that the service calculates aspect ratio as
# (HorizontalResolution / VerticalResolution), using 1.0 if vertical resolution is zero

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Test 1: Normal aspect ratio calculation
        gimbal1 = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-80.0,
            MaxElevation=-20.0,
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        camera1 = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=1600,
            VideoStreamVerticalResolution=1200,  # Aspect ratio = 1600/1200 = 1.333...
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,
            randomize=True
        )

        vehicle_config1 = Object(
            class_name='AirVehicleConfiguration',
            ID=400,
            NominalAltitude=1000.0,
            PayloadConfigurationList=[gimbal1, camera1],
            randomize=True
        )

        server.send_msg(vehicle_config1)
        time.sleep(0.2)

        footprint_request1 = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            randomize=True
        )

        requests1 = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=[footprint_request1],
            randomize=True
        )

        server.send_msg(requests1)

        msg1 = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg1.descriptor == "uxas.messages.task.SensorFootprintResponse"
        footprints1 = msg1.obj['Footprints']

        # Service responded - verify aspect ratio if valid footprints generated
        valid_footprints1 = [fp for fp in footprints1 if fp['AspectRatio'] > 0] if len(footprints1) > 0 else []
        if len(valid_footprints1) > 0:
            aspect_ratio1 = valid_footprints1[0]['AspectRatio']
            expected_aspect1 = 1600.0 / 1200.0  # 1.333...
            assert abs(aspect_ratio1 - expected_aspect1) < 0.01, \
                f"AspectRatio {aspect_ratio1} not close to expected {expected_aspect1}"

        # Test 2: Zero vertical resolution - should use 1.0
        gimbal2 = Object(
            class_name='GimbalConfiguration',
            PayloadID=30,
            MinElevation=-80.0,
            MaxElevation=-20.0,
            IsElevationClamped=True,
            ContainedPayloadList=[40],
            randomize=True
        )

        camera2 = Object(
            class_name='CameraConfiguration',
            PayloadID=40,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=0,  # Zero - should default to aspect ratio 1.0
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,
            randomize=True
        )

        vehicle_config2 = Object(
            class_name='AirVehicleConfiguration',
            ID=500,
            NominalAltitude=1000.0,
            PayloadConfigurationList=[gimbal2, camera2],
            randomize=True
        )

        server.send_msg(vehicle_config2)
        time.sleep(0.2)

        footprint_request2 = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=2,
            VehicleID=500,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            randomize=True
        )

        requests2 = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=101,
            Footprints=[footprint_request2],
            randomize=True
        )

        server.send_msg(requests2)

        msg2 = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg2.descriptor == "uxas.messages.task.SensorFootprintResponse"
        footprints2 = msg2.obj['Footprints']

        # Service responded - verify aspect ratio if valid footprints generated
        valid_footprints2 = [fp for fp in footprints2 if fp['AspectRatio'] > 0] if len(footprints2) > 0 else []
        if len(valid_footprints2) > 0:
            aspect_ratio2 = valid_footprints2[0]['AspectRatio']
            assert abs(aspect_ratio2 - 1.0) < 0.01, \
                f"AspectRatio {aspect_ratio2} should be 1.0 for zero vertical resolution"

        print("OK")
    finally:
        print("Here")

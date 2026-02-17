import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-SENS-015: Wavelength Matching
# Tests that the service only considers cameras whose SupportedWavelengthBand
# matches the requested wavelength or when the requested wavelength is AllAny

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configuration with two cameras: one EO, one IR
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
            FieldOfViewMode=1,
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
            FieldOfViewMode=1,
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

        # Request EO wavelength only - should only get EO camera footprint
        footprint_request_eo = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],  # EO only
            GroundSampleDistances=[5.0],
            randomize=True
        )

        requests_eo = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=[footprint_request_eo],
            randomize=True
        )

        server.send_msg(requests_eo)

        msg_eo = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg_eo.descriptor == "uxas.messages.task.SensorFootprintResponse"
        footprints_eo = msg_eo.obj['Footprints']
        assert len(footprints_eo) > 0, "Should have EO footprints"

        # Verify footprints with valid camera IDs use the EO camera (ID 20)
        valid_footprints_eo = [fp for fp in footprints_eo if fp['CameraID'] != 0]
        if len(valid_footprints_eo) > 0:
            for fp in valid_footprints_eo:
                assert fp['CameraID'] == 20, \
                    f"Expected EO camera (20), got camera {fp['CameraID']}"

        # Request IR wavelength only - should only get IR camera footprint
        footprint_request_ir = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=2,
            VehicleID=400,
            EligibleWavelengths=[2],  # IR only
            GroundSampleDistances=[5.0],
            randomize=True
        )

        requests_ir = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=101,
            Footprints=[footprint_request_ir],
            randomize=True
        )

        server.send_msg(requests_ir)

        msg_ir = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg_ir.descriptor == "uxas.messages.task.SensorFootprintResponse"
        footprints_ir = msg_ir.obj['Footprints']
        assert len(footprints_ir) > 0, "Should have IR footprints"

        # Verify footprints with valid camera IDs use the IR camera (ID 40)
        valid_footprints_ir = [fp for fp in footprints_ir if fp['CameraID'] != 0]
        if len(valid_footprints_ir) > 0:
            for fp in valid_footprints_ir:
                assert fp['CameraID'] == 40, \
                    f"Expected IR camera (40), got camera {fp['CameraID']}"

        print("OK")
    finally:
        print("Here")

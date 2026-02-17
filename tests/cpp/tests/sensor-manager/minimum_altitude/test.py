import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-SENS-001: Minimum Altitude Check
# Tests that the service only calculates footprints when the altitude
# is at least MIMIMUM_ASSIGNED_ALTITUDE_M (10.0 meters)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configuration
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
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,
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

        # Send footprint request with altitude below minimum (< 10.0 meters)
        # This should result in no footprints generated
        footprint_request_low = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[5.0],  # Below MIMIMUM_ASSIGNED_ALTITUDE_M (10.0)
            randomize=True
        )

        requests_low = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=[footprint_request_low],
            randomize=True
        )

        server.send_msg(requests_low)

        msg_low = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg_low.descriptor == "uxas.messages.task.SensorFootprintResponse"
        assert msg_low.obj['ResponseID'] == 100

        # Should have no valid footprints for altitude < 10.0 meters
        footprints_low = msg_low.obj['Footprints']
        # Service returns footprints but with zero values when altitude is too low
        valid_footprints_low = [fp for fp in footprints_low if fp['CameraID'] != 0]
        assert len(valid_footprints_low) == 0, \
            f"Should not generate valid footprints for altitude < 10m, got {len(valid_footprints_low)}"

        # Now send request with altitude >= 10.0 meters
        footprint_request_ok = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=2,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[100.0],  # Above minimum
            randomize=True
        )

        requests_ok = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=101,
            Footprints=[footprint_request_ok],
            randomize=True
        )

        server.send_msg(requests_ok)

        msg_ok = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg_ok.descriptor == "uxas.messages.task.SensorFootprintResponse"
        assert msg_ok.obj['ResponseID'] == 101

        # Should have footprints for altitude >= 10.0 meters
        footprints_ok = msg_ok.obj['Footprints']
        assert len(footprints_ok) > 0, \
            "Should generate footprints for altitude >= 10m"

        print("OK")
    finally:
        print("Here")

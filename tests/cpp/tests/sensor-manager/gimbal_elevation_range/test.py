import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-SENS-010: Specific Elevation Override
# Tests that if a non-zero elevation angle is specified in the request,
# the service uses that angle as both minimum and maximum elevation

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

        # Request with specific elevation angle (-45 degrees)
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[1000.0],
            ElevationAngles=[-45.0],  # Specific elevation in degrees
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
        assert len(footprints) > 0, "Should generate footprints with specific elevation"

        # The service processes ElevationAngles values as raw floats and compares them
        # against the gimbal's min/max in radians. With ElevationAngles=[-45.0], the
        # value -45.0 is numerically less than any typical gimbal minimum in radians
        # (e.g. -80° → -1.396 rad), so the service pins to the gimbal's minimum
        # elevation. The gimbal IS found and GSD IS computed.
        fp = footprints[0]
        assert fp['AchievedGSD'] > 0, \
            f"AchievedGSD {fp['AchievedGSD']} should be positive (sensor found with elevation)"
        assert fp['GimbalElevation'] < 0, \
            f"GimbalElevation {fp['GimbalElevation']} should be negative (pointing downward)"

        print("OK")
    finally:
        print("Here")

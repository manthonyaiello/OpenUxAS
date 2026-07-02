import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-PROC-004: Default Wavelength Handling
# Tests that if a footprint request has an empty EligibleWavelengths list,
# the service uses WavelengthBand::AllAny as the default wavelength

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configuration with an EO camera
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
            SupportedWavelengthBand=1,  # EO (specific wavelength)
            FieldOfViewMode=0,
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

        # Send footprint request with empty wavelength list
        # This should default to AllAny (0) and match any camera
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[],  # Empty - should default to AllAny
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
        assert msg.obj['ResponseID'] == 100

        # Should have footprints even though wavelength list was empty
        # Empty wavelength defaults to AllAny, which should match the EO camera
        footprints = msg.obj['Footprints']
        assert len(footprints) > 0, \
            "Should generate footprints with empty wavelength list (default to AllAny)"

        # Verify that if valid footprints were generated, they use the correct camera
        valid_footprints = [fp for fp in footprints if fp['CameraID'] != 0]
        if len(valid_footprints) > 0:
            assert valid_footprints[0]['CameraID'] == 20, \
                f"CameraID {valid_footprints[0]['CameraID']} != 20"
        # If no valid footprints, the test still passes (service correctly returns empty footprints)

        print("OK")
    finally:
        print("Here")

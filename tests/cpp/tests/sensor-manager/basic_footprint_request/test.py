import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Create bridge configuration
bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send an AirVehicleConfiguration with a simple camera/gimbal setup
        # This tests REQ-STATE-001 and REQ-STATE-002
        gimbal_payload = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-90.0,  # degrees
            MaxElevation=-10.0,  # degrees
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        camera_payload = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=50.0,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,  # EO (Electro-Optical)
            FieldOfViewMode=0,  # Continuous
            randomize=True
        )

        vehicle_config = Object(
            class_name='AirVehicleConfiguration',
            ID=400,
            NominalAltitude=1000.0,  # meters AGL
            PayloadConfigurationList=[gimbal_payload, camera_payload],
            randomize=True
        )

        server.send_msg(vehicle_config)
        time.sleep(0.2)

        # Send a SensorFootprintRequests message
        # This tests REQ-PROC-001, REQ-PROC-008, and basic footprint generation
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],  # EO
            GroundSampleDistances=[0.1],
            AglAltitudes=[1000.0],
            ElevationAngles=[-90.0],
            randomize=True
        )

        requests = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=[footprint_request],
            randomize=True
        )

        server.send_msg(requests)

        # Wait for response - tests REQ-RESP-001 and REQ-RESP-003
        msg = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        # Verify we got a response
        assert msg.descriptor == "uxas.messages.task.SensorFootprintResponse"

        # Verify ResponseID matches RequestID (REQ-PROC-002)
        assert msg.obj['ResponseID'] == 100, \
            f"ResponseID {msg.obj['ResponseID']} != RequestID 100"

        # Verify we got at least one footprint back
        footprints = msg.obj['Footprints']
        assert len(footprints) > 0, "No footprints generated"

        # Verify the footprint has the correct VehicleID (REQ-PROC-010)
        assert footprints[0]['VehicleID'] == 400, \
            f"VehicleID {footprints[0]['VehicleID']} != 400"

        # Verify the footprint has the correct FootprintResponseID (REQ-PROC-009)
        assert footprints[0]['FootprintResponseID'] == 1, \
            f"FootprintResponseID {footprints[0]['FootprintResponseID']} != 1"

        print("OK")
    finally:
        print("Here")

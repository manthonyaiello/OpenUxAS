import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-PROC-003: Unknown Entity Handling
# Tests that if a footprint request references an entity ID not in the
# configuration map, the service skips that request without generating a footprint

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configuration for entity 400 only
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

        # Send footprint requests for both known (400) and unknown (999) entities
        footprint_request_known = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,  # Known entity
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            randomize=True
        )

        footprint_request_unknown = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=2,
            VehicleID=999,  # Unknown entity
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            randomize=True
        )

        requests = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=[footprint_request_known, footprint_request_unknown],
            randomize=True
        )

        server.send_msg(requests)

        msg = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg.descriptor == "uxas.messages.task.SensorFootprintResponse"
        assert msg.obj['ResponseID'] == 100

        # Should have exactly one footprint (only for entity 400, not 999)
        footprints = msg.obj['Footprints']
        assert len(footprints) >= 1, "Should have at least one footprint for known entity"

        # Verify all footprints are for entity 400, none for 999
        for footprint in footprints:
            assert footprint['VehicleID'] == 400, \
                f"Found footprint for unexpected vehicle {footprint['VehicleID']}"

        # Verify we have footprint for request ID 1 but not 2
        request_ids = set(fp['FootprintResponseID'] for fp in footprints)
        assert 1 in request_ids, "Should have footprint for known entity request"
        assert 2 not in request_ids, "Should not have footprint for unknown entity request"

        print("OK")
    finally:
        print("Here")

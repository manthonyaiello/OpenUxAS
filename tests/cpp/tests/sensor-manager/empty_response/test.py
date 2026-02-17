import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-RESP-004: Empty Response Handling
# Tests that even if no footprints can be generated, the service sends
# a SensorFootprintResponse message with an empty Footprints list

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send footprint request for a vehicle that doesn't exist
        # This should result in an empty response
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=999,  # Non-existent vehicle
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

        # Should still get a response even though no footprints can be generated
        msg = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg.descriptor == "uxas.messages.task.SensorFootprintResponse"

        # Verify ResponseID matches RequestID (REQ-PROC-002)
        assert msg.obj['ResponseID'] == 100, \
            f"ResponseID {msg.obj['ResponseID']} != 100"

        # Verify empty footprints list (REQ-RESP-004)
        footprints = msg.obj['Footprints']
        assert len(footprints) == 0, \
            f"Expected empty footprints list, got {len(footprints)} footprints"

        print("OK")
    finally:
        print("Here")

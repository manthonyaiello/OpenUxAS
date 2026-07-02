import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Test REQ-PROC-008 and REQ-PROC-010: Multiple vehicles in single request
# Tests that the service can handle requests for multiple vehicles
# and correctly assigns VehicleID in each SensorFootprint

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Send configurations for two vehicles
        for vehicle_id in [400, 500]:
            gimbal = Object(
                class_name='GimbalConfiguration',
                PayloadID=vehicle_id + 10,
                MinElevation=-80.0,
                MaxElevation=-20.0,
                IsElevationClamped=True,
                ContainedPayloadList=[vehicle_id + 20],
                randomize=True
            )

            camera = Object(
                class_name='CameraConfiguration',
                PayloadID=vehicle_id + 20,
                MinHorizontalFieldOfView=10.0,
                MaxHorizontalFieldOfView=30.0,
                VideoStreamHorizontalResolution=1920,
                VideoStreamVerticalResolution=1080,
                SupportedWavelengthBand=1,
                FieldOfViewMode=0,
                randomize=True
            )

            vehicle_config = Object(
                class_name='AirVehicleConfiguration',
                ID=vehicle_id,
                NominalAltitude=1000.0,
                PayloadConfigurationList=[gimbal, camera],
                randomize=True
            )

            server.send_msg(vehicle_config)
            time.sleep(0.1)

        # Send footprint requests for both vehicles
        requests_list = []
        for idx, vehicle_id in enumerate([400, 500]):
            footprint_request = Object(
                class_name='task.FootprintRequest',
                FootprintRequestID=idx + 1,
                VehicleID=vehicle_id,
                EligibleWavelengths=[1],
                GroundSampleDistances=[0.1],
                AglAltitudes=[1000.0],
                ElevationAngles=[-80.0],
                randomize=True
            )
            requests_list.append(footprint_request)

        requests = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=requests_list,
            randomize=True
        )

        server.send_msg(requests)

        msg = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg.descriptor == "uxas.messages.task.SensorFootprintResponse"
        assert msg.obj['ResponseID'] == 100

        footprints = msg.obj['Footprints']
        assert len(footprints) >= 2, f"Should have at least 2 footprints, got {len(footprints)}"

        # Verify we got footprints for both vehicles
        vehicle_ids = set(fp['VehicleID'] for fp in footprints)
        assert 400 in vehicle_ids, "Missing footprint for vehicle 400"
        assert 500 in vehicle_ids, "Missing footprint for vehicle 500"

        # Verify FootprintResponseID matches FootprintRequestID
        for fp in footprints:
            if fp['VehicleID'] == 400:
                assert fp['FootprintResponseID'] == 1, \
                    f"Vehicle 400 FootprintResponseID {fp['FootprintResponseID']} != 1"
            elif fp['VehicleID'] == 500:
                assert fp['FootprintResponseID'] == 2, \
                    f"Vehicle 500 FootprintResponseID {fp['FootprintResponseID']} != 2"

        print("OK")
    finally:
        print("Here")

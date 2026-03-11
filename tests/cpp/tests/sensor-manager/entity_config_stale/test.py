import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Exposes SM-5: EntityConfiguration is not updated when a second message
# arrives for the same entity ID.
#
# C++ stores configurations with:
#   m_idVsEntityConfiguration.insert(std::make_pair(id, config))
#
# std::unordered_map::insert is a no-op when the key already exists.  If a
# new EntityConfiguration arrives for an entity that was already configured,
# the new configuration is silently discarded.  All subsequent footprint
# calculations use the stale first-seen configuration.
#
# This test sends two configurations for entity 400:
#   Config 1: NominalAltitude = 200.0 m
#   Config 2: NominalAltitude = 1500.0 m
#
# Both footprint requests use AglAltitudes=[] so the service falls back to the
# entity's NominalAltitude.  After the second config is sent, C++ still uses
# NominalAltitude=200 because insert() did not update the map.
#
# See CPP_BUGS.md SM-5 for full analysis.

FIRST_NOMINAL_ALTITUDE = 200.0
SECOND_NOMINAL_ALTITUDE = 1500.0

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # --- Configuration 1: NominalAltitude = 200 m ---
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
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,
            randomize=True
        )

        vehicle_config1 = Object(
            class_name='AirVehicleConfiguration',
            ID=400,
            NominalAltitude=FIRST_NOMINAL_ALTITUDE,
            PayloadConfigurationList=[gimbal1, camera1],
            randomize=True
        )

        server.send_msg(vehicle_config1)
        time.sleep(0.2)

        # First request: AglAltitudes=[] → C++ uses NominalAltitude=200.
        # randomize=True gives ElevationAngles in [0,1), bypassing the SM-4 bug
        # and allowing a full gimbal sweep.
        footprint_request1 = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[],
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
        footprints1 = [fp for fp in msg1.obj['Footprints'] if fp['AchievedGSD'] > 0]
        assert len(footprints1) > 0, \
            "First request: should find a sensor with NominalAltitude=200"
        first_altitude = footprints1[0]['AglAltitude']
        assert abs(first_altitude - FIRST_NOMINAL_ALTITUDE) < 1.0, \
            f"First request: expected AglAltitude ≈ {FIRST_NOMINAL_ALTITUDE}, got {first_altitude}"

        # --- Configuration 2: NominalAltitude = 1500 m, same entity ID ---
        gimbal2 = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-80.0,
            MaxElevation=-20.0,
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        camera2 = Object(
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

        vehicle_config2 = Object(
            class_name='AirVehicleConfiguration',
            ID=400,                            # Same entity ID as config 1
            NominalAltitude=SECOND_NOMINAL_ALTITUDE,
            PayloadConfigurationList=[gimbal2, camera2],
            randomize=True
        )

        server.send_msg(vehicle_config2)
        time.sleep(0.2)

        # Second request: AglAltitudes=[] → should use NominalAltitude=1500 if
        # the configuration was updated.  Bug SM-5: insert() is a no-op, so
        # the service still uses NominalAltitude=200.
        footprint_request2 = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=2,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[],
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
        footprints2 = [fp for fp in msg2.obj['Footprints'] if fp['AchievedGSD'] > 0]
        assert len(footprints2) > 0, \
            "Second request: should find a sensor"
        second_altitude = footprints2[0]['AglAltitude']

        # Bug SM-5: the second EntityConfiguration was not stored (insert() no-op),
        # so the service uses the first config's NominalAltitude=200 for this request.
        # A correct implementation would report AglAltitude ≈ 1500.
        assert abs(second_altitude - FIRST_NOMINAL_ALTITUDE) < 1.0, \
            (f"Bug SM-5: expected AglAltitude ≈ {FIRST_NOMINAL_ALTITUDE} "
             f"(stale config, insert() did not update), got {second_altitude}")

        print("OK")
    finally:
        pass

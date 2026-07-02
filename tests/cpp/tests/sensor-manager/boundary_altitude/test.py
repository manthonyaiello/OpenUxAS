import os
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Boundary test for the assigned-altitude gate (SUBTYPES.md D6). Both
# implementations refuse to plan below 10 m; Ada additionally refuses
# above 100 km (the Assigned_Altitude_M subtype), where C++ happily
# computes footprints.
#
# One request with five altitudes, elevation pinned at the gimbal
# minimum, single discrete FOV candidate:
#   idx  altitude    C++             Ada
#   0    9.999 m     degenerate      degenerate    (agree, below gate)
#   1    10.0 m      computed        computed      (agree, at lower bound)
#   2    99999.0 m   computed        computed      (agree, before bound)
#   3    100000.0 m  computed        computed      (agree, at upper bound)
#   4    100001.0 m  computed        degenerate    (D6, across the bound)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

ALTITUDES = [9.999, 10.0, 99999.0, 100000.0, 100001.0]

with Server(bridge_cfg=bridge_cfg) as server:
    try:
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
            FieldOfViewMode=1,  # Discrete
            DiscreteHorizontalFieldOfViewList=[15.0],
            MinHorizontalFieldOfView=15.0,
            MaxHorizontalFieldOfView=15.0,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
            randomize=True
        )
        vehicle = Object(
            class_name='AirVehicleConfiguration',
            ID=400,
            NominalAltitude=1000.0,
            PayloadConfigurationList=[gimbal, camera],
            randomize=True
        )
        server.send_msg(vehicle)
        time.sleep(0.2)

        request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[0.001],
            AglAltitudes=ALTITUDES,
            ElevationAngles=[-80.0],
            randomize=True
        )
        requests = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=[request],
            randomize=True
        )
        server.send_msg(requests)

        msg = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )
        footprints = msg.obj['Footprints']
        assert len(footprints) == len(ALTITUDES), \
            f"Expected {len(ALTITUDES)} footprints, got {len(footprints)}"

        # Below the 10 m gate: degenerate in both implementations.
        assert footprints[0]['AchievedGSD'] == 0.0
        assert footprints[0]['AglAltitude'] == 0.0

        # At and inside the bounds: computed in both implementations.
        for i, alt in ((1, 10.0), (2, 99999.0), (3, 100000.0)):
            assert footprints[i]['AchievedGSD'] > 0.0, \
                f"[{i}] altitude {alt} should compute a footprint"
            assert abs(footprints[i]['AglAltitude'] - alt) < 0.01 * alt

        if os.environ.get('UXAS_IMPL') == 'ada':
            # Across the 100 km bound: the Ada gate fails.
            assert footprints[4]['AchievedGSD'] == 0.0
            assert footprints[4]['AglAltitude'] == 0.0
        else:
            # C++ checks only the lower bound.
            assert footprints[4]['AchievedGSD'] > 0.0
            assert abs(footprints[4]['AglAltitude'] - 100001.0) < 1000.0

        print("OK")
    finally:
        pass

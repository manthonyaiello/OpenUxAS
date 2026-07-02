import os
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Boundary test for discrete FOV validity, (0, 179] degrees
# (SUBTYPES.md D5). C++ enumerates whatever the configuration supplies:
# a zero FOV yields GSD 0, a negative FOV yields a negative GSD, and a
# 180 deg FOV yields a footprint width of 2 * slant * tan(90 deg), about
# 3E19 m. Ada skips out-of-range candidates; a camera with none
# contributes only the degenerate all-zero footprint.
#
# One vehicle per single-entry discrete FOV list, elevation pinned at the
# gimbal minimum (both implementations evaluate there):
#   idx  FOV        C++                          Ada
#   0    0.001 deg  computed                     computed        (agree)
#   1    179.0 deg  computed (width 232,690 m)   computed        (agree)
#   2    0.0 deg    GSD 0, real slant/center     degenerate      (D5)
#   3    180.0 deg  width ~3.3E19 m              degenerate      (D5)
#   4    -5.0 deg   negative GSD                 degenerate      (D5)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

CASES = [(401, 0.001), (402, 179.0), (403, 0.0), (404, 180.0), (405, -5.0)]

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        for vid, fov in CASES:
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
                DiscreteHorizontalFieldOfViewList=[fov],
                MinHorizontalFieldOfView=15.0,
                MaxHorizontalFieldOfView=15.0,
                VideoStreamHorizontalResolution=1920,
                VideoStreamVerticalResolution=1080,
                SupportedWavelengthBand=1,
                randomize=True
            )
            vehicle = Object(
                class_name='AirVehicleConfiguration',
                ID=vid,
                NominalAltitude=1000.0,
                PayloadConfigurationList=[gimbal, camera],
                randomize=True
            )
            server.send_msg(vehicle)
        time.sleep(0.3)

        requests = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=[
                Object(
                    class_name='task.FootprintRequest',
                    FootprintRequestID=i + 1,
                    VehicleID=vid,
                    EligibleWavelengths=[1],
                    GroundSampleDistances=[0.01],
                    AglAltitudes=[1000.0],
                    ElevationAngles=[-80.0],
                    randomize=True
                )
                for i, (vid, _) in enumerate(CASES)
            ],
            randomize=True
        )
        server.send_msg(requests)

        msg = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )
        footprints = msg.obj['Footprints']
        assert len(footprints) == len(CASES), \
            f"Expected {len(CASES)} footprints, got {len(footprints)}"

        # Just inside the valid range: both implementations agree.
        assert footprints[0]['AchievedGSD'] > 0.0
        assert abs(footprints[0]['HorizontalFOV'] - 0.001) < 1e-6
        assert abs(footprints[1]['HorizontalFOV'] - 179.0) < 1e-3
        # Width at 179 deg: 2 * slant * tan(89.5 deg), about 232,700 m.
        assert abs(footprints[1]['WidthCenter'] - 232690.0) < 200.0

        if os.environ.get('UXAS_IMPL') == 'ada':
            # Out-of-range candidates are skipped: degenerate footprints.
            for i in (2, 3, 4):
                assert footprints[i]['CameraID'] == 0, \
                    f"[{i}] expected degenerate footprint"
                assert footprints[i]['SlantRangeToCenter'] == 0.0
        else:
            # C++ computes through the nonsense: a footprint of width
            # ~6.8E15 m for the 180 deg FOV, and a negative GSD and
            # width for the -5 deg FOV.
            assert footprints[2]['AchievedGSD'] == 0.0        # FOV 0
            assert footprints[2]['SlantRangeToCenter'] > 1000.0
            assert footprints[3]['WidthCenter'] > 1.0e12       # FOV 180
            assert footprints[4]['AchievedGSD'] < 0.0          # FOV -5
            assert footprints[4]['WidthCenter'] < 0.0

        print("OK")
    finally:
        pass

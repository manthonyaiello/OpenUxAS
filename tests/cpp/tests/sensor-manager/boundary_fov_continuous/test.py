import os
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Boundary test for continuous FOV ranges (SUBTYPES.md D5). C++
# enumerates Min..Max in 5-degree steps with no validity filter, so a
# range reaching below zero produces zero and negative FOV candidates
# (GSD <= 0) which its integer-truncated comparison then keeps. Ada
# enumerates the same 5-degree grid (anchored at the wire minimum) but
# only its points inside (0, 179] degrees.
#
# One vehicle per (Min, Max) range, elevation pinned at the gimbal
# minimum, desired GSD 0.01 m/px:
#   idx  range        C++ selects                 Ada selects
#   0    [5, 5]       FOV 5                       FOV 5        (agree)
#   1    [10, 5]      empty range: degenerate     degenerate   (agree)
#   2    [-10, 30]    FOV -10 (negative GSD)      FOV 5        (D5)
#   3    [0, 10]      FOV 0 (GSD 0)               FOV 5        (D5)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

CASES = [(401, 5.0, 5.0), (402, 10.0, 5.0),
         (403, -10.0, 30.0), (404, 0.0, 10.0)]

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        for vid, fov_min, fov_max in CASES:
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
                FieldOfViewMode=0,  # Continuous
                MinHorizontalFieldOfView=fov_min,
                MaxHorizontalFieldOfView=fov_max,
                DiscreteHorizontalFieldOfViewList=[15.0],
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
                for i, (vid, _, _) in enumerate(CASES)
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

        # Single-point range: agree.
        assert abs(footprints[0]['HorizontalFOV'] - 5.0) < 1e-6
        assert footprints[0]['AchievedGSD'] > 0.0
        # Inverted range: no candidates in either implementation.
        assert footprints[1]['CameraID'] == 0
        assert footprints[1]['AchievedGSD'] == 0.0

        if os.environ.get('UXAS_IMPL') == 'ada':
            # The 5-degree grid anchored at the wire minimum, filtered to
            # (0, 179]: first valid candidate is 5 deg in both cases, and
            # the desired 0.01 GSD keeps it.
            assert abs(footprints[2]['HorizontalFOV'] - 5.0) < 1e-6
            assert footprints[2]['AchievedGSD'] > 0.0
            assert abs(footprints[3]['HorizontalFOV'] - 5.0) < 1e-6
            assert footprints[3]['AchievedGSD'] > 0.0
        else:
            # C++ keeps the first candidate of the same-integer-bucket
            # deltas: FOV -10 (negative GSD) and FOV 0 (GSD 0).
            assert abs(footprints[2]['HorizontalFOV'] - (-10.0)) < 1e-6
            assert footprints[2]['AchievedGSD'] < 0.0
            assert abs(footprints[3]['HorizontalFOV'] - 0.0) < 1e-6
            assert footprints[3]['AchievedGSD'] == 0.0

        print("OK")
    finally:
        pass

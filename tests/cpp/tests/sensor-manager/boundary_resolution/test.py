import os
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Boundary test for video resolutions (SUBTYPES.md D7 and the aspect
# ratio of a zero-width camera). Ada clamps resolutions to 65,536 px per
# axis; C++ uses them as-is. A camera with zero horizontal resolution
# gets aspect ratio 0.0 in C++ (0/vertical) but 1.0 in Ada; the computed
# geometry is identical because C++ guards a zero aspect ratio by using
# the horizontal FOV as the vertical FOV.
#
# One vehicle per resolution pair, elevation pinned at the gimbal
# minimum, single discrete FOV candidate (15 deg), altitude 1000 m:
#   idx  resolution      C++                         Ada
#   0    65536 x 65536   GSD 0.00406                 same          (agree)
#   1    1920 x 0        GSD = slant, aspect 1.0     same          (agree)
#   2    70000 x 70000   GSD 0.00380                 GSD 0.00406   (D7)
#   3    0 x 1080        GSD = slant, aspect 0.0     aspect 1.0    (D7 note)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

CASES = [(401, 65536, 65536), (402, 1920, 0),
         (403, 70000, 70000), (404, 0, 1080)]

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        for vid, h_res, v_res in CASES:
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
                VideoStreamHorizontalResolution=h_res,
                VideoStreamVerticalResolution=v_res,
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
                    GroundSampleDistances=[0.001],
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

        # At the 65,536 px bound: GSD = slant * sin(15 deg in rad / 65536)
        # = 1015.43 * 3.9953E-6, about 0.004057 m/px, in both.
        assert abs(footprints[0]['AchievedGSD'] - 0.004057) < 1e-5
        assert abs(footprints[0]['AspectRatio'] - 1.0) < 1e-6

        # Zero vertical resolution: aspect 1.0 and worst-case GSD (the
        # full slant range) in both implementations.
        assert abs(footprints[1]['AspectRatio'] - 1.0) < 1e-6
        assert abs(footprints[1]['AchievedGSD'] - 1015.43) < 0.5

        if os.environ.get('UXAS_IMPL') == 'ada':
            # Across the bound: Ada clamps 70,000 px to 65,536 px.
            assert abs(footprints[2]['AchievedGSD'] - 0.004057) < 1e-5
            # Zero horizontal resolution: Ada reports aspect 1.0.
            assert abs(footprints[3]['AspectRatio'] - 1.0) < 1e-6
        else:
            # C++ uses the raw 70,000 px: GSD = 1015.43 * (0.2618/70000).
            assert abs(footprints[2]['AchievedGSD'] - 0.003798) < 1e-5
            # C++ computes aspect = 0 / 1080 = 0.0 for a zero-width
            # camera (its footprint math then guards the zero).
            assert abs(footprints[3]['AspectRatio'] - 0.0) < 1e-6
        # Geometry of the zero-width camera is identical either way.
        assert abs(footprints[3]['AchievedGSD'] - 1015.43) < 0.5

        print("OK")
    finally:
        pass

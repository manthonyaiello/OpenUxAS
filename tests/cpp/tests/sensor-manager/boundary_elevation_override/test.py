import os
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Boundary test for the requested-elevation override (SUBTYPES.md D2/D3 and
# the preserved C++ dead path).
#
# UXTASK.xml documents ElevationAngles as degrees. The C++ service compares
# the raw degree value against gimbal limits in radians, so:
#   - values <= the gimbal minimum in *radians* pin to the gimbal minimum,
#   - values in (gimbal_min_rad, 0) are used raw *as radians*,
#   - values in [0, 0.001) hit the dead path (no footprint computed),
#   - values >= 0.001 are ignored (full-range sweep).
# Ada converts degrees to radians and clamps into the working range
# [-179 deg, -1 deg] and the gimbal's own range.
#
# Gimbal range [-80, -20] deg; single discrete FOV (15 deg) so candidate
# selection is trivial; altitude 1000 m.
#
# Requested elevations (degrees), one footprint each:
#   idx  request    C++ evaluates at         Ada evaluates at
#   0    -80.0      -80 deg                  -80 deg      (agree)
#   1    -45.0      -80 deg (pin bug)        -45 deg      (D2)
#   2    -20.0      -80 deg (pin bug)        -20 deg      (D2)
#   3    -10.0      -80 deg (pin bug)        -20 deg      (D2+D3: gimbal max)
#   4    -0.5       -0.5 rad = -28.6 deg     -20 deg      (D2: raw-as-radians)
#   5    -0.0005    -0.0005 rad = -0.03 deg  -20 deg      (slant blows up in C++)
#   6    0.0        dead path                dead path    (agree, degenerate)
#   7    0.0005     dead path                dead path    (agree, degenerate)
#   8    0.001      ignored, full sweep      ignored      (agree: GSD 0.01 makes
#   9    5.0        ignored, full sweep      ignored       first candidate best)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

ELEVATIONS = [-80.0, -45.0, -20.0, -10.0, -0.5, -0.0005,
              0.0, 0.0005, 0.001, 5.0]

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
            GroundSampleDistances=[0.01],
            AglAltitudes=[1000.0],
            ElevationAngles=ELEVATIONS,
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
        assert len(footprints) == len(ELEVATIONS), \
            f"Expected {len(ELEVATIONS)} footprints, got {len(footprints)}"

        def elev(i):
            return footprints[i]['GimbalElevation']

        # Dead-path sentinels: degenerate all-zero footprints either way.
        for i in (6, 7):
            assert footprints[i]['AchievedGSD'] == 0.0, \
                f"[{i}] request {ELEVATIONS[i]} should be the dead path"

        # Ignored positive elevations: full sweep; the desired GSD (0.01)
        # is below every achievable GSD, so the first candidate (-80 deg)
        # wins under both selection rules.
        for i in (8, 9):
            assert abs(elev(i) - (-80.0)) < 0.01, \
                f"[{i}] full sweep should select -80 deg, got {elev(i)}"

        assert abs(elev(0) - (-80.0)) < 0.01, f"[0] expected -80, got {elev(0)}"

        if os.environ.get('UXAS_IMPL') == 'ada':
            # Ada: degrees are converted and clamped into the gimbal range.
            assert abs(elev(1) - (-45.0)) < 0.01, f"[1] got {elev(1)}"
            assert abs(elev(2) - (-20.0)) < 0.01, f"[2] got {elev(2)}"
            assert abs(elev(3) - (-20.0)) < 0.01, f"[3] got {elev(3)}"
            assert abs(elev(4) - (-20.0)) < 0.01, f"[4] got {elev(4)}"
            assert abs(elev(5) - (-20.0)) < 0.01, f"[5] got {elev(5)}"
        else:
            # C++: degree requests compared against radian limits.
            assert abs(elev(1) - (-80.0)) < 0.01, f"[1] got {elev(1)}"
            assert abs(elev(2) - (-80.0)) < 0.01, f"[2] got {elev(2)}"
            assert abs(elev(3) - (-80.0)) < 0.01, f"[3] got {elev(3)}"
            # -0.5 "degrees" used raw as radians: -28.65 deg.
            assert abs(elev(4) - (-28.6479)) < 0.01, f"[4] got {elev(4)}"
            # -0.0005 raw radians: -0.0286 deg; slant range explodes to
            # about 2.0E6 m for a 1000 m altitude.
            assert abs(elev(5) - (-0.0286)) < 0.01, f"[5] got {elev(5)}"
            assert footprints[5]['SlantRangeToCenter'] > 1.0e6, \
                f"[5] C++ slant should blow up, got " \
                f"{footprints[5]['SlantRangeToCenter']}"

        print("OK")
    finally:
        pass

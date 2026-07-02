import os
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Boundary test for the lower working-elevation bound (-179 deg,
# SUBTYPES.md D4) and the CMASI +/-180 deg wire clamp. C++ raises a
# gimbal minimum to -179 deg only when it is below -180 deg (-pi in
# radians), so minima in [-180, -179) survive: at exactly -180 deg the
# trigonometry degenerates (sin(pi) is one rounding ulp from zero) and
# the footprint is garbage. Ada clamps both ends into [-179, -1] deg.
#
# Fixed gimbals (min == max), full single-point sweep via an ignored
# positive requested elevation:
#   idx  fixed elevation  C++ evaluates at   Ada evaluates at
#   0    -179.0 deg       -179 deg           -179 deg   (agree)
#   1    -179.5 deg       -179.5 deg         -179 deg   (D4)
#   2    -180.0 deg       -180 deg (garbage) -179 deg   (D4)
#   3    -200.0 deg       -179 deg           -179 deg   (agree: both clamp)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

CASES = [(401, -179.0), (402, -179.5), (403, -180.0), (404, -200.0)]

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        for vid, elev_deg in CASES:
            gimbal = Object(
                class_name='GimbalConfiguration',
                PayloadID=10,
                MinElevation=elev_deg,
                MaxElevation=elev_deg,  # fixed-elevation gimbal
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
                    ElevationAngles=[5.0],  # ignored: full (single) sweep
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

        # At the bound (idx 0) and beyond the CMASI range (idx 3, clamped
        # by both implementations): -179 deg, looking backward, so the
        # horizontal distance to the footprint center is negative.
        for i in (0, 3):
            assert abs(footprints[i]['GimbalElevation'] - (-179.0)) < 0.01, \
                f"[{i}] got {footprints[i]['GimbalElevation']}"
            assert footprints[i]['HorizontalToCenter'] < -50000.0, \
                f"[{i}] backward-looking center expected, got " \
                f"{footprints[i]['HorizontalToCenter']}"

        if os.environ.get('UXAS_IMPL') == 'ada':
            for i in (1, 2):
                assert abs(footprints[i]['GimbalElevation'] - (-179.0)) < 0.01
                assert abs(footprints[i]['SlantRangeToCenter'] - 57298.7) \
                    < 10.0
        else:
            # C++ keeps -179.5 deg: slant = 1000 / sin(0.5 deg).
            assert abs(footprints[1]['GimbalElevation'] - (-179.5)) < 0.01
            assert abs(footprints[1]['SlantRangeToCenter'] - 114593.0) < 20.0
            # At exactly -180 deg the C++ result is garbage whose exact
            # value depends on its double-comparison tolerance; only pin
            # the reported angle.
            assert abs(footprints[2]['GimbalElevation'] - (-180.0)) < 0.01

        print("OK")
    finally:
        pass

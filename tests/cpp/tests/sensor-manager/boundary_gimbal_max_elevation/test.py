import os
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Boundary test for the upper working-elevation bound (-1 deg,
# SUBTYPES.md D4). C++ clamps a gimbal max elevation to -1 deg only when
# it is strictly positive, so a fixed gimbal at -0.5 deg computes a
# footprint there (slant range ~114.6 x altitude and growing without
# bound as the angle approaches 0). Ada clamps both sweep ends into the
# working range [-179 deg, -1 deg].
#
# Three vehicles with fixed gimbals (min == max), evaluated via a
# positive requested elevation (>= 0.001 deg is ignored by both
# implementations, giving the full -- here single-point -- sweep):
#   idx  fixed elevation  C++ evaluates at  Ada evaluates at
#   0    -1.5 deg         -1.5 deg          -1.5 deg   (agree)
#   1    -1.0 deg         -1.0 deg          -1.0 deg   (agree, at bound)
#   2    -0.5 deg         -0.5 deg          -1.0 deg   (D4)

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

CASES = [(401, -1.5), (402, -1.0), (403, -0.5)]

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

        # Before and at the bound: identical in both implementations.
        assert abs(footprints[0]['GimbalElevation'] - (-1.5)) < 0.01
        assert abs(footprints[1]['GimbalElevation'] - (-1.0)) < 0.01
        # Slant = altitude / sin(1 deg) = 57,298 m at the bound.
        assert abs(footprints[1]['SlantRangeToCenter'] - 57298.7) < 10.0

        if os.environ.get('UXAS_IMPL') == 'ada':
            # Across the bound: Ada clamps to -1 deg.
            assert abs(footprints[2]['GimbalElevation'] - (-1.0)) < 0.01
            assert abs(footprints[2]['SlantRangeToCenter'] - 57298.7) < 10.0
        else:
            # C++ evaluates at -0.5 deg: slant = altitude / sin(0.5 deg).
            assert abs(footprints[2]['GimbalElevation'] - (-0.5)) < 0.01
            assert abs(footprints[2]['SlantRangeToCenter'] - 114593.0) < 20.0

        print("OK")
    finally:
        pass

import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Exposes SM-6: inconsistent zero-denominator fallback between GSD calculation
# (FindSensorFootPrint) and footprint geometry (CalculateSensorFootprint).
#
# This test relies on SM-1 (strict `<` clamp) to place the gimbal at exactly
# -180°.  With MinElevation = MaxElevation = -180°, the clamp guard:
#
#   gimbalElevationMin_rad = (gimbalElevationMin_rad < -Pi) ? ... : gimbalElevationMin_rad
#
# evaluates to FALSE (since -Pi is not strictly less than -Pi), so the loop
# starts at exactly -Pi radians.  This is the only step in the sweep.
#
# At -180°: sin(-(-Pi)) = sin(Pi) ≈ 1.22e-16 ≈ 0.
# bCompareDouble detects this as approximately zero and applies fallback values:
#
#   In FindSensorFootPrint (line 265):
#     dSlantRangeMin_m = altitudeAgl_m          <- fallback: slant = altitude
#   In CalculateSensorFootprint (line 351):
#     slantRangeToCenter_m = 0.0                <- fallback: slant = 0
#
# The GSD is therefore computed using altitudeAgl_m as the slant range
# (giving a nonzero AchievedGSD), while SlantRangeToCenter is stored as 0.0.
# These two fields are mutually inconsistent.
#
# See CPP_BUGS.md SM-6 for full analysis.

ALTITUDE_M = 1000.0

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        # Gimbal locked at exactly -180°.  IsElevationClamped=True so the
        # unclamped-gimbal override (lines 246-251) is not applied.
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-180.0,   # Exactly -Pi rad — escapes the clamp (SM-1)
            MaxElevation=-180.0,   # Same: single-step sweep at -180°
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        camera = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,  # Continuous
            randomize=True
        )

        vehicle_config = Object(
            class_name='AirVehicleConfiguration',
            ID=400,
            NominalAltitude=ALTITUDE_M,
            PayloadConfigurationList=[gimbal, camera],
            randomize=True
        )

        server.send_msg(vehicle_config)
        time.sleep(0.2)

        # randomize=True gives ElevationAngles in [0, 1), which are ≥ 0.001 and
        # therefore skip the `if (elevationAngle < 0.001)` block (SM-4), leaving
        # the gimbal range at [-180°, -180°] for the single-step sweep.
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[ALTITUDE_M],
            randomize=True
        )

        requests = Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100,
            Footprints=[footprint_request],
            randomize=True
        )

        server.send_msg(requests)

        msg = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse',
            timeout=5.0
        )

        assert msg.descriptor == "uxas.messages.task.SensorFootprintResponse"
        footprints = msg.obj['Footprints']
        assert len(footprints) > 0, "Service should emit a footprint object"

        fp = footprints[0]

        # AchievedGSD is computed using slant = altitudeAgl_m (the fallback in
        # FindSensorFootPrint), so it should be positive.
        assert fp['AchievedGSD'] > 0, \
            (f"AchievedGSD {fp['AchievedGSD']} should be > 0: "
             f"GSD was computed using altitude as slant range")

        # SlantRangeToCenter is computed using the fallback in
        # CalculateSensorFootprint, which uses 0.0 instead of altitudeAgl_m.
        # This is the SM-6 inconsistency: AchievedGSD > 0 but SlantRangeToCenter = 0.
        assert fp['SlantRangeToCenter'] == 0.0, \
            (f"Bug SM-6: expected SlantRangeToCenter=0.0 (zero-denominator fallback "
             f"in CalculateSensorFootprint differs from GSD fallback), "
             f"got {fp['SlantRangeToCenter']}")

        print("OK")
    finally:
        pass

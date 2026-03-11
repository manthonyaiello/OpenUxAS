import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Exposes SM-7: divide-by-zero in CalculateSensorFootprint when
# VideoStreamHorizontalResolution = 0 and VideoStreamVerticalResolution > 0.
#
# The aspect ratio guard only checks for zero VERTICAL resolution:
#
#   dAspectRatio = (verticalResolution == 0) ? 1.0
#                                            : (double)horizontal / (double)vertical;
#
# When horizontal = 0 and vertical > 0, dAspectRatio = 0.0.
# CalculateSensorFootprint then divides:
#
#   double verticalFov_rad = horizantalFov_rad / dAspectRatio;  // ÷ 0.0 = ±Inf
#
# Under IEEE 754 this yields ±infinity rather than a crash.  The subsequent
# angle clamps (> 0.0 → 0.0; < -π → -π) recover finite values, so the
# service continues running and produces a footprint.  However, the footprint
# geometry fields are numerically garbage: the vertical FOV was treated as
# infinite, so leading/trailing edge distances are derived from degenerate
# angle values.
#
# AchievedGSD is unaffected by the divide-by-zero: it is computed before
# CalculateSensorFootprint is called, using videoStreamResolutionMin = 0
# → alpha_rad = π/2 → GSD = slant_range (the "worst case" fallback).
#
# See CPP_BUGS.md SM-7 for full analysis.

ALTITUDE_M = 1000.0

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

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

        # Key configuration: horizontal resolution = 0, vertical = 1080.
        # This triggers the unguarded dAspectRatio = 0.0 path.
        camera = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            MinHorizontalFieldOfView=10.0,
            MaxHorizontalFieldOfView=30.0,
            VideoStreamHorizontalResolution=0,    # ← triggers SM-7
            VideoStreamVerticalResolution=1080,   # nonzero: guard not applied
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

        # randomize=True gives ElevationAngles in [0, 1), bypassing SM-4.
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

        # The service must not crash: IEEE 754 silent Inf/NaN propagation and
        # the angle clamps keep the computation alive.
        assert msg.descriptor == "uxas.messages.task.SensorFootprintResponse"
        footprints = msg.obj['Footprints']
        assert len(footprints) > 0, "Service should not crash and must emit a footprint"

        fp = footprints[0]

        # AchievedGSD is computed using videoStreamResolutionMin=0 → alpha=π/2
        # → GSD = slant_range = altitude / sin(|elevation|).  This is the
        # "worst-case" fallback: the service finds a sensor but reports the
        # slant range as the GSD (ignoring resolution entirely).
        assert fp['AchievedGSD'] > 0, \
            (f"AchievedGSD {fp['AchievedGSD']} should be positive: GSD falls back "
             f"to slant range when horizontal resolution is 0")

        # HorizontalToLeadingEdge is derived from tan(-gimbalAngleMax_rad).
        # gimbalAngleMax_rad was clamped to 0.0 (via Inf path), so
        # tan(-0.0)=0 → bCompareDouble catches it → HorizontalToLeadingEdge = 0.0.
        # This is a symptom of the garbage geometry produced by the divide-by-zero.
        assert fp['HorizontalToLeadingEdge'] == 0.0, \
            (f"Bug SM-7: HorizontalToLeadingEdge={fp['HorizontalToLeadingEdge']} "
             f"should be 0.0 (gimbalAngleMax clamped to 0 via Inf vertical FOV)")

        print("OK")
    finally:
        pass

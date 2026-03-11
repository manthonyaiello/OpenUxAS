import time
import math
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Exposes SM-4 Sub-bug B: a specified ElevationAngles value is silently ignored
# because it is compared against a radians variable without unit conversion.
#
# LMCP defines ElevationAngles with Units="deg".  C++ reads the float directly
# and at line 255 compares it against gimbalElevationMin_rad (in radians):
#
#   gimbalElevationMin_rad =
#       (elevationAngle <= gimbalElevationMin_rad) ? gimbalElevationMin_rad
#                                                  : elevationAngle;
#
# With ElevationAngles=[-30.0] and a gimbal whose MinElevation=-60°:
#   gimbalElevationMin_rad = -60 * pi/180 ≈ -1.047 rad
#   comparison: -30.0 (degrees) <= -1.047 (radians) → TRUE  (−30 < −1.047 numerically)
#
# So gimbalElevationMin_rad is left at -1.047 rad (= -60°), and gimbalElevationMax_rad
# is also set to -1.047.  The loop sweeps exactly one step at the gimbal's MINIMUM
# elevation (-60°), not at the requested -30°.
#
# Ada note: Ada's Compute_Elevation_Range deliberately replicates this unit mismatch
# (see sensor_manager.adb lines 63–68, 562–564: "intentional unit mismatch replicating
# C++ behaviour").  Both implementations pin to the gimbal minimum for this input, so
# there is no b2b divergence and no b2b.yaml is needed for this test.
#
# See CPP_BUGS.md SM-4 for full analysis.

GIMBAL_MIN_DEG = -60.0
GIMBAL_MAX_DEG = -10.0
REQUESTED_ELEVATION_DEG = -30.0

# Expected C++ output: gimbal pins to its minimum elevation (-60°)
EXPECTED_GIMBAL_ELEVATION_DEG = GIMBAL_MIN_DEG

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=GIMBAL_MIN_DEG,
            MaxElevation=GIMBAL_MAX_DEG,
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
            NominalAltitude=1000.0,
            PayloadConfigurationList=[gimbal, camera],
            randomize=True
        )

        server.send_msg(vehicle_config)
        time.sleep(0.2)

        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[1000.0],
            ElevationAngles=[REQUESTED_ELEVATION_DEG],  # -30° in degrees
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
        assert len(footprints) > 0, "Service should produce a footprint"

        fp = footprints[0]

        # Bug SM-4B: the requested elevation (-30°) is compared against
        # gimbalElevationMin_rad (≈ -1.047 rad) without converting degrees to
        # radians.  Since −30 < −1.047 numerically, the comparison is TRUE and
        # the requested angle is discarded; the gimbal pins to its minimum (-60°).
        #
        # A correct implementation would output GimbalElevation ≈ -30.0°.
        assert fp['AchievedGSD'] > 0, \
            f"AchievedGSD {fp['AchievedGSD']} should be positive (sensor IS found, just at wrong angle)"

        # C++ pins to the gimbal minimum (-60°), not the requested -30°.
        # Allow 1° tolerance for floating-point rounding in the conversion.
        assert abs(fp['GimbalElevation'] - EXPECTED_GIMBAL_ELEVATION_DEG) < 1.0, \
            (f"Bug SM-4B: expected GimbalElevation ≈ {EXPECTED_GIMBAL_ELEVATION_DEG}° "
             f"(pinned to gimbal minimum due to degrees-vs-radians comparison), "
             f"got {fp['GimbalElevation']}")

        print("OK")
    finally:
        pass

import math
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Explore C++ behavior with HFOV = 190 degrees.
#
# horizantalFov_rad = 190 * M_PI / 180 = 19*M_PI/18.
# 0.5 * horizantalFov_rad = 95 degrees (= Pi/2 + Pi/36).
# tan(95 deg) = tan(Pi/2 + Pi/36) = -cot(Pi/36) ~ -11.43  (second quadrant: negative).
#
# WidthCenter = 2 * slantRangeToCenter * tan(95 deg)
#            = 2 * slantRangeToCenter * (-11.43)
#            -> NEGATIVE.
#
# This is a degenerate result: a 190 deg horizontal FOV on a downward-facing
# sensor has no geometrically meaningful ground footprint (the sensor sees sky
# on both sides beyond 90 deg from boresight), but C++ silently computes and
# stores a negative width.
#
# Same gimbal as wide_fov_179: -180 .. 10, full-range sweep.

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-180.0,
            MaxElevation=10.0,
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )

        camera = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,
            DiscreteHorizontalFieldOfViewList=[190.0],
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
            ElevationAngles=[1.0],
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
        assert len(footprints) == 1

        fp = footprints[0]
        print(f"HFOV=190 deg:")
        print(f"  HorizontalFOV       = {fp['HorizontalFOV']}")
        print(f"  GimbalElevation     = {fp['GimbalElevation']} deg")
        print(f"  AchievedGSD         = {fp['AchievedGSD']} m")
        print(f"  SlantRangeToCenter  = {fp['SlantRangeToCenter']} m")
        print(f"  WidthCenter         = {fp['WidthCenter']} m")
        print(f"  HorizontalToCenter  = {fp['HorizontalToCenter']} m")

        assert fp['AchievedGSD'] > 0, "Should find a sensor footprint (GSD uses slant*sin, not tan)"
        # tan(95 deg) ~ -11.43; WidthCenter = 2 * slant * (-11.43) is NEGATIVE.
        # This is the degenerate case: C++ silently produces a negative footprint width.
        assert fp['WidthCenter'] < 0, \
            f"tan(95 deg) < 0, so WidthCenter should be negative; got {fp['WidthCenter']}"

        print("OK")
    finally:
        pass

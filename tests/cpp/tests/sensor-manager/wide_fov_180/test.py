import math
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Explore C++ behavior with HFOV = 180 degrees.
#
# horizantalFov_rad = 180 * M_PI / 180 = M_PI (the double approximation of Pi).
# tan(M_PI / 2) is called; since M_PI/2 is not exactly Pi/2 (Pi is irrational),
# IEEE 754 returns a large finite value (~1.633e16) rather than +infinity.
# WidthCenter = 2 * slantRange * 1.633e16, which is astronomically large but
# representable as a float (float max ~ 3.4e38).
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
            DiscreteHorizontalFieldOfViewList=[180.0],
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
        print(f"HFOV=180 deg:")
        print(f"  HorizontalFOV       = {fp['HorizontalFOV']}")
        print(f"  GimbalElevation     = {fp['GimbalElevation']} deg")
        print(f"  AchievedGSD         = {fp['AchievedGSD']} m")
        print(f"  SlantRangeToCenter  = {fp['SlantRangeToCenter']} m")
        print(f"  WidthCenter         = {fp['WidthCenter']} m")
        print(f"  HorizontalToCenter  = {fp['HorizontalToCenter']} m")
        print(f"  math.isfinite(WidthCenter) = {math.isfinite(fp['WidthCenter'])}")
        print(f"  math.isinf(WidthCenter)    = {math.isinf(fp['WidthCenter'])}")

        assert fp['AchievedGSD'] > 0, "Should find a sensor footprint"
        # tan(M_PI/2) in IEEE 754 is a large positive finite number (~1.633e16),
        # because M_PI/2 is not the exact mathematical Pi/2.
        # WidthCenter overflows float (float max ~3.4e38 but 2*slant*1.633e16 ~ 4.6e19 is OK).
        assert fp['WidthCenter'] > 0 or math.isinf(fp['WidthCenter']), \
            f"WidthCenter should be positive (possibly inf); got {fp['WidthCenter']}"

        print("OK")
    finally:
        pass

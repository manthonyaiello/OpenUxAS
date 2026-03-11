import math
import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Explore C++ behavior with HFOV = 179 degrees.
#
# Gimbal range -180 .. 10 degrees (MaxElevation=10 clamped to -1 deg in C++).
# MinElevation=-180 stays as -Pi (C++ clamp only fires for strictly less than -Pi).
#
# ElevationAngles=[1.0]: value >= 0.001 so C++ does NOT pin to a specific angle;
# the full gimbal range is swept in 5-degree steps.
#
# At the best elevation (where |desired_gsd - achieved_gsd| is minimised) the
# footprint geometry is computed.  For HFOV just under 180 deg, tan(HFOV/2) is
# large but finite and positive, so WidthCenter should be a large positive number.

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
            SupportedWavelengthBand=1,   # EO
            FieldOfViewMode=1,           # Discrete (C++ FOVOperationMode::Discrete=1)
            DiscreteHorizontalFieldOfViewList=[179.0],
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
            ElevationAngles=[1.0],  # >= 0.001: full range sweep, not pinned
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
        print(f"HFOV=179 deg:")
        print(f"  HorizontalFOV       = {fp['HorizontalFOV']}")
        print(f"  GimbalElevation     = {fp['GimbalElevation']} deg")
        print(f"  AchievedGSD         = {fp['AchievedGSD']} m")
        print(f"  SlantRangeToCenter  = {fp['SlantRangeToCenter']} m")
        print(f"  WidthCenter         = {fp['WidthCenter']} m")
        print(f"  HorizontalToCenter  = {fp['HorizontalToCenter']} m")

        assert fp['AchievedGSD'] > 0, "Should find a sensor footprint"
        assert fp['WidthCenter'] > 0, \
            f"tan(89.5 deg) > 0, so WidthCenter should be positive; got {fp['WidthCenter']}"
        # tan(89.5 deg) ~ 114.6; at 1000 m altitude and ~45 deg elevation
        # slant ~ 1414 m, so WidthCenter ~ 2 * 1414 * 114.6 ~ 324,000 m
        assert fp['WidthCenter'] > 1000, \
            f"WidthCenter should be very large for 179 deg FOV; got {fp['WidthCenter']}"

        print("OK")
    finally:
        pass

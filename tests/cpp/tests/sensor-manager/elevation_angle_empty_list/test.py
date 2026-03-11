import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

# Exposes SM-4 Sub-bug A: empty ElevationAngles list produces AchievedGSD = 0.
#
# When ElevationAngles is empty, C++ pushes sentinel 0.0 (FindSensorFootPrint
# line 177).  The sentinel satisfies the `if (elevationAngle < 0.001)` branch
# (line 252), which treats it as a "specified elevation" of 0°.  This sets:
#
#   gimbalElevationMin_rad = max(negative_min, 0.0) = 0.0
#   gimbalElevationMax_rad = 0.0
#
# The outer guard `if (gimbalElevationMin_rad < 0.0)` at line 258 then
# evaluates to FALSE, so the entire gimbal sweep is skipped and AchievedGSD
# stays at the default 0.0.
#
# The intended behaviour (per the LMCP field comment "uses an optimal elevation
# angle for achieving max GSD") would be a full sweep yielding AchievedGSD > 0.
#
# See CPP_BUGS.md SM-4 for full analysis.

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

        # Explicitly pass an empty ElevationAngles list.  C++ will push sentinel
        # 0.0 and then skip the gimbal sweep entirely.
        footprint_request = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1,
            VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[1000.0],
            ElevationAngles=[],  # Empty list — triggers the bug
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
        assert len(footprints) > 0, "Service always emits a footprint object per request"

        fp = footprints[0]

        # Bug SM-4A: the sentinel 0.0 causes the gimbal sweep to be skipped, so
        # AchievedGSD remains at the default 0.0 even though the gimbal is perfectly
        # capable of pointing at the ground.  A correct implementation would do a
        # full sweep and return a positive AchievedGSD.
        assert fp['AchievedGSD'] == 0.0, \
            (f"Bug SM-4A: expected AchievedGSD=0.0 (gimbal sweep skipped by sentinel), "
             f"got {fp['AchievedGSD']}")

        print("OK")
    finally:
        pass
